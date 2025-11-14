// Dual-belt treadmill controller for Arduino Mega 2560

#include <Arduino.h>
#include <EEPROM.h>
#include <Encoder.h>

constexpr uint8_t PWM_MAX = 255;
constexpr uint16_t CONTROL_INTERVAL_US = 5000;
constexpr float CONTROL_PERIOD_S = CONTROL_INTERVAL_US / 1000000.0f;
constexpr uint32_t TELEMETRY_INTERVAL_MS = 50;

constexpr int32_t ENCODER_CPR = 2048;  // 每转脉冲数
constexpr float MAX_RPM = 800.0f;

struct MotorPins {
  uint8_t pwm;
  uint8_t dir;
  uint8_t sleep;
  uint8_t fault;
};

constexpr MotorPins MOTOR_PINS[2] = {
    {3, 4, 5, 30},   // M1
    {6, 7, 8, 32}    // M2 (暂不处理)
};

constexpr uint8_t ENCODER_PIN_A[2] = {22, 26};
constexpr uint8_t ENCODER_PIN_B[2] = {24, 28};
constexpr uint8_t NFault_PIN = 10;

enum class CommandMode { INDEPENDENT, DIFFERENTIAL };

struct CalibrationData {
  uint16_t magic = 0xA5A5;        // Magic number to detect valid data
  float kp[2];
  float ki[2];
  float kd[2];
  float feedForward[2];
};

struct MotorState {
  float targetRpm = 0.0f;
  float actualRpm = 0.0f;
  float integral = 0.0f;
  float lastError = 0.0f;
  float controlEffort = 0.0f;
  bool enabled = true;
};

CalibrationData calib;
MotorState motors[2];
CommandMode controlMode = CommandMode::INDEPENDENT;

Encoder enc1(ENCODER_PIN_A[0], ENCODER_PIN_B[0]);
Encoder enc2(ENCODER_PIN_A[1], ENCODER_PIN_B[1]);

bool driverHealthy[2] = {true, true};
unsigned long lastControlMicros = 0;
unsigned long lastTelemetryMs = 0;

// Profile queue
struct ProfileStep {
  float rpm[2];
  uint32_t durationMs;
};
ProfileStep profileQueue[8];
uint8_t profileHead = 0;
uint8_t profileTail = 0;
bool profileActive = false;
unsigned long profileStepMs = 0;

// ------------------------ Utility ------------------------
inline void setMotorEnable(uint8_t idx, bool enable) {
  digitalWrite(MOTOR_PINS[idx].sleep, enable ? HIGH : LOW);
  motors[idx].enabled = enable;
}

inline void applyMotorPwm(uint8_t idx, float effort) {
  effort = constrain(effort, -1.0f, 1.0f);
  bool dir = effort >= 0.0f;
  uint8_t duty = static_cast<uint8_t>(fabsf(effort) * PWM_MAX);
  digitalWrite(MOTOR_PINS[idx].dir, dir ? HIGH : LOW);
  analogWrite(MOTOR_PINS[idx].pwm, duty);
}

int32_t readAndZeroEncoder(uint8_t idx) {
  int32_t count = (idx == 0) ? enc1.read() : enc2.read();
  if (count != 0) {  // 只有当计数不为零时才清零
    (idx == 0) ? enc1.write(0) : enc2.write(0);
  }
  return count;
}

// 修改后的 countsToRpm 函数 - 接受电机索引
float countsToRpm(int32_t counts, uint8_t idx) {
  if (counts == 0) return 0.0f;  // 避免除零错误
  
  float revs = static_cast<float>(counts) / ENCODER_CPR;
  float rpm = (revs / CONTROL_PERIOD_S) * 60.0f;
  
  // 添加滤波，避免瞬时噪声
  static float lastRpm[2] = {0.0f, 0.0f};
  float filteredRpm = lastRpm[idx] * 0.7f + rpm * 0.3f;
  lastRpm[idx] = filteredRpm;
  
  return filteredRpm;
}



// ------------------------ EEPROM ------------------------
void loadCalibration()
{
    CalibrationData tmp;   // 用临时变量接 EEPROM 的值

    EEPROM.get(0, tmp);
    if (tmp.magic != 0xA5A5) {
        // 首次或损坏，写入默认
        tmp.magic = 0xA5A5;
        tmp.kp[0] = 0.15f;    // 增强比例增益
        tmp.kp[1] = 0.15f;    // M2参数暂时保留
        tmp.ki[0] = 0.30f;    // 增强积分增益
        tmp.ki[1] = 0.30f;
        tmp.kd[0] = 0.0005f;  // 增强微分增益
        tmp.kd[1] = 0.0005f;
        tmp.feedForward[0] = 0.00040f;  // 增加前馈补偿
        tmp.feedForward[1] = 0.00040f;

        // 写回 EEPROM
        EEPROM.put(0, tmp);
    }

    calib = tmp;
}


void saveCalibration() {
  EEPROM.put(0, calib);
}

// ------------------------ Safety ------------------------
void disableAllMotors() {
  for (uint8_t i = 0; i < 2; ++i) {
    applyMotorPwm(i, 0.0f);
    setMotorEnable(i, false);
  }
}

void checkSafetyInputs() {
  // ⚠️ DEBUG 版本：完全忽略故障引脚，强制使能两个电机
  driverHealthy[0] = true;
  driverHealthy[1] = true;
  bool globalFault = false;

  for (uint8_t i = 0; i < 2; ++i) {
    setMotorEnable(i, true);   // 把 SLEEP 引脚一直拉高
  }
}


// ------------------------ Control ------------------------
void runControl() {
  for (uint8_t i = 0; i < 2; ++i) {
    // ① 读编码器、算实际转速
    int32_t delta = readAndZeroEncoder(i);
    motors[i].actualRpm = countsToRpm(delta, i);  // 传入电机索引

    // ② PID 误差计算
    float error = motors[i].targetRpm - motors[i].actualRpm;
    
    // 如果误差过大，重置积分项以避免积分饱和
    if (abs(error) > 50.0f) {
      motors[i].integral = 0.0f;
    }
    
    motors[i].integral += error * CONTROL_PERIOD_S;
    motors[i].integral = constrain(motors[i].integral, -MAX_RPM, MAX_RPM);
    float derivative = (error - motors[i].lastError) / CONTROL_PERIOD_S;
    motors[i].lastError = error;

    float pid = calib.kp[i] * error
              + calib.ki[i] * motors[i].integral
              + calib.kd[i] * derivative;
    float ff  = calib.feedForward[i] * motors[i].targetRpm;

    // 计算原始控制输出
    float rawEffort = (pid + ff) / MAX_RPM;

    // 添加启动辅助：仅在静止且目标非零时提供额外推力
    if (motors[i].targetRpm != 0.0f && abs(motors[i].actualRpm) < 1.0f) {
        const float STARTUP_BOOST = 0.30f;
        if (motors[i].targetRpm > 0) {
            rawEffort = max(rawEffort, STARTUP_BOOST);
        } else {
            rawEffort = min(rawEffort, -STARTUP_BOOST);
        }
    }

    motors[i].controlEffort = constrain(rawEffort, -1.0f, 1.0f);

    // 使能逻辑
    if (!motors[i].enabled) {
      applyMotorPwm(i, 0.0f);
    } else {
      applyMotorPwm(i, motors[i].controlEffort);
    }
  }
}

// ------------------------ Profile ------------------------
void enqueueProfileStep(float rpm0, float rpm1, uint32_t duration) {
  uint8_t next = (profileTail + 1) % 8;
  if (next == profileHead) {
    Serial.println(F("ERR,PROFILE_FULL"));
    return;
  }
  profileQueue[profileTail] = {{rpm0, rpm1}, duration};
  profileTail = next;
}

void handleProfile() {
  if (!profileActive && profileHead != profileTail) {
    auto &step = profileQueue[profileHead];
    motors[0].targetRpm = constrain(step.rpm[0], -MAX_RPM, MAX_RPM);
    motors[1].targetRpm = constrain(step.rpm[1], -MAX_RPM, MAX_RPM);
    profileActive = true;
    profileStepMs = millis();
  } else if (profileActive) {
    auto &step = profileQueue[profileHead];
    if (millis() - profileStepMs >= step.durationMs) {
      profileHead = (profileHead + 1) % 8;
      profileActive = false;
    }
  }
}

void setTargetsFromCommand(float left, float right) {
  if (controlMode == CommandMode::INDEPENDENT) {
    motors[0].targetRpm = constrain(left, -MAX_RPM, MAX_RPM);
    motors[1].targetRpm = constrain(right, -MAX_RPM, MAX_RPM);
  } else {
    float u = (left + right) * 0.5f;
    float v = (left - right) * 0.5f;
    motors[0].targetRpm = constrain(u + v, -MAX_RPM, MAX_RPM);
    motors[1].targetRpm = constrain(u - v, -MAX_RPM, MAX_RPM);
  }
}

// ------------------------ Serial ------------------------
String serialBuffer;

void handleCommand(const String &line) {
  if (line.startsWith("SPD")) {
    Serial.print(F("DBG,SPD_CMD,"));
    Serial.println(line);
    int p1 = line.indexOf(',');
    int p2 = line.indexOf(',', p1 + 1);
    if (p1 < 0 || p2 < 0) return;
    float a = line.substring(p1 + 1, p2).toFloat();
    float b = line.substring(p2 + 1).toFloat();
    setTargetsFromCommand(a, b);
    profileHead = profileTail;
    profileActive = false;
  }
  else if (line.startsWith("SEQ")) {
    int p1 = line.indexOf(',');
    int p2 = line.indexOf(',', p1 + 1);
    int p3 = line.indexOf(',', p2 + 1);
    if (p1 < 0 || p2 < 0 || p3 < 0) return;
    uint32_t d = line.substring(p1 + 1, p2).toInt();
    float a = line.substring(p2 + 1, p3).toFloat();
    float b = line.substring(p3 + 1).toFloat();
    enqueueProfileStep(a, b, d);
  }
  else if (line.startsWith("MODE")) {
    controlMode = line.endsWith("DIFF") ? CommandMode::DIFFERENTIAL : CommandMode::INDEPENDENT;
  }
  else if (line.startsWith("CFG")) {
    int comma = line.indexOf(',', 4);
    if (comma < 0) return;
    String key = line.substring(4, comma);
    float val = line.substring(comma + 1).toFloat();
    if (key == "KP1") calib.kp[0] = val;
    else if (key == "KP2") calib.kp[1] = val;
    else if (key == "KI1") calib.ki[0] = val;
    else if (key == "KI2") calib.ki[1] = val;
    else if (key == "KD1") calib.kd[0] = val;
    else if (key == "KD2") calib.kd[1] = val;
    else if (key == "FF1") calib.feedForward[0] = val;
    else if (key == "FF2") calib.feedForward[1] = val;
    saveCalibration();
  }
}

void pollSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialBuffer.length() > 0) {
        handleCommand(serialBuffer);
        serialBuffer = "";
      }
    } else {
      serialBuffer += c;
    }
  }
}

void publishTelemetry() {
  Serial.print(F("TEL,"));
  Serial.print(millis());
  Serial.print(',');
  Serial.print(motors[0].targetRpm, 2);
  Serial.print(',');
  Serial.print(motors[0].actualRpm, 2);
  Serial.print(',');
  Serial.print(motors[1].targetRpm, 2);
  Serial.print(',');
  Serial.print(motors[1].actualRpm, 2);
  Serial.print(',');
  Serial.print(driverHealthy[0]);
  Serial.print(',');
  Serial.print(driverHealthy[1]);
  Serial.print(',');
  Serial.print(false);
  Serial.print(',');
  Serial.println(profileActive);
}

// ------------------------ Setup & Loop ------------------------
void configurePins() {
  pinMode(NFault_PIN, INPUT_PULLUP);
  for (uint8_t i = 0; i < 2; ++i) {
    pinMode(MOTOR_PINS[i].dir, OUTPUT);
    pinMode(MOTOR_PINS[i].sleep, OUTPUT);
    pinMode(MOTOR_PINS[i].pwm, OUTPUT);        // Critical: set as output
    pinMode(MOTOR_PINS[i].fault, INPUT_PULLUP);

    analogWrite(MOTOR_PINS[i].pwm, 0);         // Critical: clear floating state
    setMotorEnable(i, true);
  }
}

void setup() {
  Serial.begin(115200);
  loadCalibration();
  configurePins();

  // Extra safety: ensure PWM is zero
  analogWrite(MOTOR_PINS[0].pwm, 0);
  analogWrite(MOTOR_PINS[1].pwm, 0);

  lastControlMicros = micros();
  lastTelemetryMs = millis();
  Serial.println(F("INFO,ARDUINO_MEGA_TREADMILL_READY"));
}

void loop() {
  unsigned long now = micros();
  if (now - lastControlMicros >= CONTROL_INTERVAL_US) {
    lastControlMicros += CONTROL_INTERVAL_US;
    checkSafetyInputs();
    handleProfile();
    runControl();
  }

  pollSerial();

  now = millis();
  if (now - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryMs = now;
    publishTelemetry();
  }
}