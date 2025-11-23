// Dual-belt treadmill controller for Arduino Mega 2560

#include <Arduino.h>
#include <EEPROM.h>
#include <Encoder.h>

constexpr uint8_t PWM_MAX = 255;
constexpr uint16_t CONTROL_INTERVAL_US = 5000;
constexpr float CONTROL_PERIOD_S = CONTROL_INTERVAL_US / 1000000.0f;
constexpr uint32_t TELEMETRY_INTERVAL_MS = 50;
constexpr float RPM_SCALE = 0.6f;  


constexpr int32_t ENCODER_CPR = 200;  
constexpr float MAX_RPM = 800.0f;

struct MotorPins {
  uint8_t pwm;
  uint8_t dir;
  uint8_t sleep;
  uint8_t fault;
};

constexpr MotorPins MOTOR_PINS[2] = {
    {3, 4, 5, 30}, // M1
    {6, 7, 8, 32}  // M2
};

constexpr uint8_t ENCODER_PIN_A[2] = {22, 26};
constexpr uint8_t ENCODER_PIN_B[2] = {24, 28};
constexpr uint8_t NFault_PIN = 10;

enum class CommandMode { INDEPENDENT, DIFFERENTIAL };

struct CalibrationData {
  uint16_t magic = 0xA5A5; // Magic number to detect valid data
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
constexpr uint8_t MAX_PROFILE_STEPS = 64;
struct ProfileStep {
  float rpm[2];
  uint32_t durationMs;
};
ProfileStep profileQueue[MAX_PROFILE_STEPS];
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
  if (idx == 1) {
    dir = !dir;
  }

  uint8_t duty = static_cast<uint8_t>(fabsf(effort) * PWM_MAX);
  digitalWrite(MOTOR_PINS[idx].dir, dir ? HIGH : LOW);
  analogWrite(MOTOR_PINS[idx].pwm, duty);
}

int32_t readAndZeroEncoder(uint8_t idx) {
  int32_t count = (idx == 0) ? enc1.read() : enc2.read();
  if (count != 0) {
    (idx == 0) ? enc1.write(0) : enc2.write(0);
  }
  return count;
}

float countsToRpm(int32_t counts, uint8_t idx) {
  if (counts == 0) return 0.0f;  

  float revs = static_cast<float>(counts) / ENCODER_CPR;
  float rpm = (revs / CONTROL_PERIOD_S) * 60.0f;

  
  rpm *= RPM_SCALE;   
  rpm = fabsf(rpm);   

  static float lastRpm[2] = {0.0f, 0.0f};
  float filteredRpm = lastRpm[idx] * 0.7f + rpm * 0.3f;
  lastRpm[idx] = filteredRpm;

  return filteredRpm;
}

// ------------------------ EEPROM ------------------------
void loadCalibration()
{
    CalibrationData tmp;   

    EEPROM.get(0, tmp);
    if (tmp.magic != 0xA5A5) {
        
        tmp.magic = 0xA5A5;
        tmp.kp[0] = 0.15f;    
        tmp.kp[1] = 0.15f;    
        tmp.ki[0] = 0.30f;   
        tmp.ki[1] = 0.30f;
        tmp.kd[0] = 0.0005f;  
        tmp.kd[1] = 0.0005f;
        tmp.feedForward[0] = 0.00040f; 
        tmp.feedForward[1] = 0.00040f;

        
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
  
  driverHealthy[0] = true;
  driverHealthy[1] = true;
  bool globalFault = false;

  for (uint8_t i = 0; i < 2; ++i) {
    setMotorEnable(i, true);   
  }
}

// ------------------------ Control ------------------------
void runControl() {
  for (uint8_t i = 0; i < 2; ++i) {
    
    int32_t delta = readAndZeroEncoder(i);
    motors[i].actualRpm = countsToRpm(delta, i);  

    float error = motors[i].targetRpm - motors[i].actualRpm;

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

    float rawEffort = (pid + ff) / MAX_RPM;

    if (motors[i].targetRpm != 0.0f && abs(motors[i].actualRpm) < 1.0f) {
        const float STARTUP_BOOST = 0.30f;
        if (motors[i].targetRpm > 0) {
            rawEffort = max(rawEffort, STARTUP_BOOST);
        } else {
            rawEffort = min(rawEffort, -STARTUP_BOOST);
        }
    }

    motors[i].controlEffort = constrain(rawEffort, -1.0f, 1.0f);

    if (!motors[i].enabled) {
      applyMotorPwm(i, 0.0f);
    } else {
      applyMotorPwm(i, motors[i].controlEffort);
    }
  }
}

// ------------------------ Profile ------------------------
void enqueueProfileStep(float rpm0, float rpm1, uint32_t duration) {
  uint8_t next = (profileTail + 1) % MAX_PROFILE_STEPS;
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
      profileHead = (profileHead + 1) % MAX_PROFILE_STEPS;
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
enum class SystemState {
  IDLE,      // Waiting for commands, manual control allowed
  UPLOADING, // Receiving profile data
  RUNNING    // Executing profile
};

SystemState systemState = SystemState::IDLE;

const uint8_t MAX_CMD_LEN = 64;
char serialBuf[MAX_CMD_LEN];
uint8_t serialPos = 0;

void parseProfileCommand(char *cmd) {
  // Format: L:1.5 R:2.0 T:3.0
  float lSpeed = 0, rSpeed = 0, duration = 0;

  char *pL = strstr(cmd, "L:");
  char *pR = strstr(cmd, "R:");
  char *pT = strstr(cmd, "T:");

  if (pL && pR && pT) {
    lSpeed = atof(pL + 2);
    rSpeed = atof(pR + 2);
    duration = atof(pT + 2);
    enqueueProfileStep(lSpeed, rSpeed, (uint32_t)(duration * 1000));
    Serial.println(F("READY"));
  }
  else {
    Serial.println(F("ERR,PARSE_FMT"));
  }
}

void handleCommand(char *cmd) {
  // 1. CRITICAL: Always check for STOP first
  if (strcmp(cmd, "STOP_TM") == 0) {
    disableAllMotors();
    motors[0].targetRpm = 0;
    motors[1].targetRpm = 0;
    profileActive = false;
    profileHead = 0;
    profileTail = 0;
    systemState = SystemState::IDLE;
    Serial.println(F("STOPPED"));
    return;
  }

  switch (systemState) {
  case SystemState::RUNNING:
    // Ignore everything else during run to prevent jitter
    if (strncmp(cmd, "SPD", 3) == 0 || strncmp(cmd, "START", 5) == 0) {
      Serial.println(F("ERR,BUSY_RUNNING"));
    }
    break;

  case SystemState::IDLE:
    if (strcmp(cmd, "START_READ") == 0) {
      profileHead = 0;
      profileTail = 0;
      profileActive = false;
      systemState = SystemState::UPLOADING;
      Serial.println(F("READY"));
    }
    else if (strncmp(cmd, "RUN_TM", 6) == 0) {
      if (profileHead != profileTail) {
        systemState = SystemState::RUNNING;
        profileActive = false;
        Serial.println(F("RUNNING"));
      }
      else {
        Serial.println(F("ERR,NO_PROFILE"));
      }
    }
    else if (strncmp(cmd, "SPD", 3) == 0) {
      // SPD,100,100
      char *p1 = strchr(cmd, ',');
      if (p1) {
        char *p2 = strchr(p1 + 1, ',');
        if (p2) {
          float a = atof(p1 + 1);
          float b = atof(p2 + 1);
          setTargetsFromCommand(a, b);
          profileHead = profileTail; // Clear profile
          profileActive = false;
        }
      }
    }
    else if (strncmp(cmd, "SEQ", 3) == 0) {
      // SEQ,1000,10.0,10.0
      char *p1 = strchr(cmd, ',');
      if (p1) {
        char *p2 = strchr(p1 + 1, ',');
        if (p2) {
          char *p3 = strchr(p2 + 1, ',');
          if (p3) {
            uint32_t d = atol(p1 + 1);
            float a = atof(p2 + 1);
            float b = atof(p3 + 1);
            enqueueProfileStep(a, b, d);
          }
        }
      }
    }
    else if (strncmp(cmd, "MODE", 4) == 0)
    {
      // MODE,DIFF or MODE,IND
      if (strstr(cmd, "DIFF"))
        controlMode = CommandMode::DIFFERENTIAL;
      else
        controlMode = CommandMode::INDEPENDENT;
    }
    else if (strncmp(cmd, "CFG", 3) == 0) {
      // CFG,KP1,0.15
      char *pKey = strchr(cmd, ',');
      if (pKey) {
        char *pVal = strchr(pKey + 1, ',');
        if (pVal) {
          *pVal = 0; // Terminate key string
          char *key = pKey + 1;
          float val = atof(pVal + 1);

          if (strcmp(key, "KP1") == 0)
            calib.kp[0] = val;
          else if (strcmp(key, "KP2") == 0)
            calib.kp[1] = val;
          else if (strcmp(key, "KI1") == 0)
            calib.ki[0] = val;
          else if (strcmp(key, "KI2") == 0)
            calib.ki[1] = val;
          else if (strcmp(key, "KD1") == 0)
            calib.kd[0] = val;
          else if (strcmp(key, "KD2") == 0)
            calib.kd[1] = val;
          else if (strcmp(key, "FF1") == 0)
            calib.feedForward[0] = val;
          else if (strcmp(key, "FF2") == 0)
            calib.feedForward[1] = val;
          saveCalibration();
        }
      }
    }
    break;

  case SystemState::UPLOADING:
    if (strncmp(cmd, "L:", 2) == 0) {
      parseProfileCommand(cmd);
    }
    else if (strcmp(cmd, "END_READ") == 0) {
      systemState = SystemState::IDLE;
      Serial.println(F("ACK"));
    }
    else {
      Serial.println(F("ERR,EXPECTED_PROFILE_DATA"));
    }
    break;
  }
}

void pollSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialPos > 0) {
        serialBuf[serialPos] = '\0';
        handleCommand(serialBuf);
        serialPos = 0;
      }
    }
    else {
      if (serialPos < MAX_CMD_LEN - 1) {
        serialBuf[serialPos++] = c;
      }
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
    pinMode(MOTOR_PINS[i].pwm, OUTPUT); // Critical: set as output
    pinMode(MOTOR_PINS[i].fault, INPUT_PULLUP);

    analogWrite(MOTOR_PINS[i].pwm, 0); // Critical: clear floating state
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