#include <Arduino.h>
#include <EEPROM.h>
#include <Encoder.h> 

/**
 * Dual-belt treadmill controller for Arduino Mega 2560.
 * Uses Paul Stoffregen's Encoder library for reliable quadrature decoding.
 * Implements:
 *  - Two DC motor channels with 8-bit PWM and direction control
 *  - Closed-loop speed regulation from dual encoders
 *  - Serial PC protocol for commands and telemetry
 *  - Safety supervision (driver faults only)
 *  - Config/persistence (EEPROM)
 *  - E-Stop and sensors removed as per requirements
 */

// ------------------------ Hardware definitions (Mega-specific) ------------------------
struct MotorPins
{
  uint8_t pwm;   // Must be a PWM pin (2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 44, 45, 46)
  uint8_t dir;
  uint8_t sleep; // Enable/Sleep pin
  uint8_t fault; // Fault pin (active-low)
};

// Define pins based on your new connection scheme
constexpr MotorPins MOTOR_PINS[2] = {
    {3, 4, 5, 30}, // Motor 1: PWM, DIR, SLP, FLT
    {6, 7, 8, 32}  // Motor 2: PWM, DIR, SLP, FLT
};

// Encoder pins (using Encoder library, no need for interrupt pins)
constexpr uint8_t ENCODER_PIN_A[2] = {24, 26}; // Encoder 1A, Encoder 2A
constexpr uint8_t ENCODER_PIN_B[2] = {25, 27}; // Encoder 1B, Encoder 2B

// Driver fault pin (if available, often shared or per-driver)
constexpr uint8_t NFault_PIN = 10; // Example - connect to driver's global fault or one of the FLT pins if shared

// ------------------------ Control parameters ------------------------
constexpr uint8_t PWM_MAX = 255; // Mega is 8-bit PWM
constexpr uint16_t CONTROL_INTERVAL_US = 5000; // 5 ms loop
constexpr float CONTROL_PERIOD_S = CONTROL_INTERVAL_US / 1'000'000.0f;
constexpr uint32_t TELEMETRY_INTERVAL_MS = 50;

// TODO: Update with actual counts/rev from encoder spec sheet
constexpr int32_t ENCODER_CPR = 2048; // Placeholder - replace with actual value
// TODO: Verify against motor limits
constexpr float MAX_RPM = 4000.0f;    // Placeholder - replace with actual value

enum class CommandMode
{
  INDEPENDENT,
  DIFFERENTIAL
};

struct CalibrationData
{
  float kp[2];
  float ki[2];
  float kd[2];
  float feedForward[2];
};

struct MotorState
{
  float targetRpm = 0.0f;
  float actualRpm = 0.0f;
  float integral = 0.0f;
  float lastError = 0.0f;
  float controlEffort = 0.0f; // -1.0 .. 1.0
  bool enabled = true;
};

CalibrationData calib;
MotorState motors[2];
CommandMode controlMode = CommandMode::INDEPENDENT;

// Create Encoder objects
Encoder enc1(ENCODER_PIN_A[0], ENCODER_PIN_B[0]);
Encoder enc2(ENCODER_PIN_A[1], ENCODER_PIN_B[1]);

// No E-Stop latch
bool driverHealthy[2] = {true, true};
unsigned long lastControlMicros = 0;
unsigned long lastTelemetryMs = 0;

// --------------- Speed profile sequence (optional) ------------------
struct ProfileStep
{
  float rpm[2];
  uint32_t durationMs;
};

ProfileStep profileQueue[8];
uint8_t profileHead = 0;
uint8_t profileTail = 0;
bool profileActive = false;
unsigned long profileStepMs = 0;

// --------------------- Utility / hardware helpers -------------------
inline void setMotorEnable(uint8_t motorIdx, bool enable)
{
  digitalWrite(MOTOR_PINS[motorIdx].sleep, enable ? HIGH : LOW);
  motors[motorIdx].enabled = enable;
}

inline void applyMotorPwm(uint8_t motorIdx, float effort)
{
  effort = constrain(effort, -1.0f, 1.0f);
  bool direction = effort >= 0.0f;
  uint8_t duty = static_cast<uint8_t>(fabsf(effort) * PWM_MAX); // 8-bit PWM (0-255)
  digitalWrite(MOTOR_PINS[motorIdx].dir, direction ? HIGH : LOW);
  analogWrite(MOTOR_PINS[motorIdx].pwm, duty); // Use analogWrite for Mega
}

// Function to read encoder counts and reset them to zero
int32_t readAndZeroEncoder(uint8_t motorIdx)
{
  int32_t count = 0;
  if (motorIdx == 0)
  {
    count = enc1.read();
    enc1.write(0); // Reset encoder position to 0
  }
  else if (motorIdx == 1)
  {
    count = enc2.read();
    enc2.write(0); // Reset encoder position to 0
  }
  return count;
}

float countsToRpm(int32_t counts)
{
  float revolutions = static_cast<float>(counts) / ENCODER_CPR;
  float rps = revolutions / CONTROL_PERIOD_S;
  return rps * 60.0f;
}

// ------------------------ Config handling (EEPROM) ---------------------------
void loadCalibration()
{
    // Load from EEPROM starting at address 0
    EEPROM.get(0, calib);
    // If it's the first run, EEPROM might be empty (all 0xFF or 0x00), initialize defaults
    if (calib.kp[0] == 0 && calib.ki[0] == 0 && calib.kd[0] == 0)
    {
        calib = {
            {0.12f, 0.12f}, // kp defaults
            {0.45f, 0.45f}, // ki defaults
            {0.0008f, 0.0008f}, // kd defaults
            {0.00025f, 0.00025f}, // feed-forward
        };
        saveCalibration(); // Write defaults to EEPROM
    }
}

void saveCalibration()
{
    // Save to EEPROM starting at address 0
    EEPROM.put(0, calib);
    // EEPROM.commit(); // Not needed on Arduino Mega, put() writes immediately
}

// ----------------------- Safety handling (Driver faults only) ----------------------------
void disableAllMotors()
{
  for (uint8_t i = 0; i < 2; ++i)
  {
    applyMotorPwm(i, 0.0f);
    setMotorEnable(i, false);
  }
}

void checkSafetyInputs()
{
  // Check individual driver faults
  driverHealthy[0] = digitalRead(MOTOR_PINS[0].fault) == HIGH; // Active-low fault
  driverHealthy[1] = digitalRead(MOTOR_PINS[1].fault) == HIGH; // Active-low fault

  // Check global driver fault (if applicable)
  bool driverGlobalFault = digitalRead(NFault_PIN) == LOW; // Active-low fault

  if (driverGlobalFault) // If any global fault occurs
  {
    disableAllMotors();
  }
  else
  {
    // Enable motors only if their individual drivers are healthy
    for (uint8_t i = 0; i < 2; ++i)
    {
      setMotorEnable(i, driverHealthy[i]);
    }
  }
}

// ------------------------ Control algorithm -------------------------
void runControl()
{
  for (uint8_t i = 0; i < 2; ++i)
  {
    int32_t delta = readAndZeroEncoder(i);
    motors[i].actualRpm = countsToRpm(delta);

    float error = motors[i].targetRpm - motors[i].actualRpm;
    motors[i].integral += error * CONTROL_PERIOD_S;
    motors[i].integral = constrain(motors[i].integral, -MAX_RPM, MAX_RPM);
    float derivative = (error - motors[i].lastError) / CONTROL_PERIOD_S;
    motors[i].lastError = error;

    float pid = calib.kp[i] * error + calib.ki[i] * motors[i].integral + calib.kd[i] * derivative;
    float ff = calib.feedForward[i] * motors[i].targetRpm;

    motors[i].controlEffort = constrain((pid + ff) / MAX_RPM, -1.0f, 1.0f);

    // Only apply PWM if motor is enabled (no fault)
    if (!motors[i].enabled)
    {
      applyMotorPwm(i, 0.0f);
    }
    else
    {
      applyMotorPwm(i, motors[i].controlEffort);
    }
  }
}

// ------------------------ PC command parser -------------------------
String serialBuffer;

void enqueueProfileStep(float rpm0, float rpm1, uint32_t duration)
{
  uint8_t next = (profileTail + 1) % 8;
  if (next == profileHead)
  {
    Serial.println(F("ERR,PROFILE_FULL"));
    return;
  }
  profileQueue[profileTail] = {{rpm0, rpm1}, duration};
  profileTail = next;
}

void handleProfile()
{
  if (!profileActive && profileHead != profileTail)
  {
    ProfileStep &step = profileQueue[profileHead];
    motors[0].targetRpm = constrain(step.rpm[0], -MAX_RPM, MAX_RPM);
    motors[1].targetRpm = constrain(step.rpm[1], -MAX_RPM, MAX_RPM);
    profileActive = true;
    profileStepMs = millis();
  }
  else if (profileActive)
  {
    ProfileStep &step = profileQueue[profileHead];
    if (millis() - profileStepMs >= step.durationMs)
    {
      profileHead = (profileHead + 1) % 8;
      profileActive = false;
    }
  }
}

void setTargetsFromCommand(float leftRpm, float rightRpm)
{
  if (controlMode == CommandMode::INDEPENDENT)
  {
    motors[0].targetRpm = constrain(leftRpm, -MAX_RPM, MAX_RPM);
    motors[1].targetRpm = constrain(rightRpm, -MAX_RPM, MAX_RPM);
  }
  else
  {
    // Differential mode: left = u + v, right = u - v
    float u = (leftRpm + rightRpm) * 0.5f;
    float v = (leftRpm - rightRpm) * 0.5f;
    motors[0].targetRpm = constrain(u + v, -MAX_RPM, MAX_RPM);
    motors[1].targetRpm = constrain(u - v, -MAX_RPM, MAX_RPM);
  }
}

void handleCommand(const String &line)
{
  if (line.startsWith("SPD"))
  {
    // Format: SPD,<rpmA>,<rpmB>
    int firstComma = line.indexOf(',');
    int secondComma = line.indexOf(',', firstComma + 1);
    if (firstComma < 0 || secondComma < 0)
      return;
    float rpmA = line.substring(firstComma + 1, secondComma).toFloat();
    float rpmB = line.substring(secondComma + 1).toFloat();
    setTargetsFromCommand(rpmA, rpmB);
    profileHead = profileTail; // Invalidate queued profile steps
    profileActive = false;
  }
  else if (line.startsWith("SEQ"))
  {
    // Format: SEQ,<duration_ms>,<rpmA>,<rpmB>
    int p1 = line.indexOf(',');
    int p2 = line.indexOf(',', p1 + 1);
    int p3 = line.indexOf(',', p2 + 1);
    if (p1 < 0 || p2 < 0 || p3 < 0)
      return;
    uint32_t duration = line.substring(p1 + 1, p2).toInt();
    float rpmA = line.substring(p2 + 1, p3).toFloat();
    float rpmB = line.substring(p3 + 1).toFloat();
    enqueueProfileStep(rpmA, rpmB, duration);
  }
  else if (line.startsWith("MODE"))
  {
    if (line.endsWith("DIFF"))
    {
      controlMode = CommandMode::DIFFERENTIAL;
    }
    else
    {
      controlMode = CommandMode::INDEPENDENT;
    }
  }
  else if (line.startsWith("CFG"))
  {
    // Format: CFG,<key>,<value>
    int comma = line.indexOf(',', 4);
    if (comma < 0)
      return;
    String key = line.substring(4, comma);
    float value = line.substring(comma + 1).toFloat();
    if (key == "KP1")
      calib.kp[0] = value;
    else if (key == "KP2")
      calib.kp[1] = value;
    else if (key == "KI1")
      calib.ki[0] = value;
    else if (key == "KI2")
      calib.ki[1] = value;
    else if (key == "KD1")
      calib.kd[0] = value;
    else if (key == "KD2")
      calib.kd[1] = value;
    else if (key == "FF1")
      calib.feedForward[0] = value;
    else if (key == "FF2")
      calib.feedForward[1] = value;
    saveCalibration(); // Save to EEPROM after any CFG change
  }
  // Removed ESTOP and RESET commands
}

void pollSerial()
{
  while (Serial.available())
  {
    char c = Serial.read();
    if (c == '\n' || c == '\r')
    {
      if (serialBuffer.length() > 0)
      {
        handleCommand(serialBuffer);
        serialBuffer = "";
      }
    }
    else
    {
      serialBuffer += c;
    }
  }
}

// ------------------------ Telemetry output (No sensors) --------------------------
void publishTelemetry()
{
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
  // Removed force, current, temp fields
  Serial.print(',');
  Serial.print(driverHealthy[0]);
  Serial.print(',');
  Serial.print(driverHealthy[1]);
  Serial.print(',');
  Serial.print(false); // No E-Stop, always false
  Serial.print(',');
  Serial.println(profileActive);
}

// ----------------------------- Setup --------------------------------
void configurePins()
{
  // pinMode(ESTOP_PIN, INPUT_PULLUP); // Removed
  pinMode(NFault_PIN, INPUT_PULLUP);

  for (uint8_t i = 0; i < 2; ++i)
  {
    pinMode(MOTOR_PINS[i].dir, OUTPUT);
    pinMode(MOTOR_PINS[i].sleep, OUTPUT);
    pinMode(MOTOR_PINS[i].fault, INPUT_PULLUP); // Driver fault is input
    // Encoder pins are handled by the Encoder library, no need to pinMode here
    setMotorEnable(i, true);
  }

  // No need to attachInterrupt for Encoder library
}

void setup()
{
  Serial.begin(115200);
  loadCalibration(); // Load saved calibration values
  configurePins();
  lastControlMicros = micros();
  lastTelemetryMs = millis();
  Serial.println(F("INFO,ARDUINO_MEGA_TREADMILL_READY_NO_ESTOP_ENCODER_LIB"));
}

// ------------------------------ Loop --------------------------------
void loop()
{
  unsigned long nowMicros = micros();
  if (nowMicros - lastControlMicros >= CONTROL_INTERVAL_US)
  {
    lastControlMicros += CONTROL_INTERVAL_US;
    checkSafetyInputs(); // Check driver faults
    if (true) // Always handle profile if no E-Stop (always true now)
    {
      handleProfile();
    }
    runControl(); // Run PID control loop
  }

  pollSerial(); // Check for incoming serial commands

  unsigned long nowMs = millis();
  if (nowMs - lastTelemetryMs >= TELEMETRY_INTERVAL_MS)
  {
    lastTelemetryMs = nowMs;
    publishTelemetry(); // Send telemetry data
  }
}