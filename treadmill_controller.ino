#include <Arduino.h>
#include <EEPROM.h> // For persistent storage on Arduino Mega

/**
 * Dual-belt treadmill controller for Arduino Mega 2560 with Speed Data Buffer.
 * Implements:
 *  - Two DC motor channels with 8-bit PWM and direction control
 *  - Closed-loop speed regulation from dual quadrature encoders
 *  - Serial PC protocol for commands and telemetry
 *  - Speed data buffer for efficient telemetry sending
 *  - Safety supervision (driver faults only)
 *  - Config/persistence (EEPROM)
 *  - E-Stop and sensors removed as per requirements
 */

// ------------------------ Hardware definitions (Based on your wiring diagram) ------------------------
struct MotorPins
{
  uint8_t pwm;   // Speed control pin (PWM capable)
  uint8_t dir;   // Direction control pin
  uint8_t sleep; // Enable/Sleep status output pin (active high)
  uint8_t fault; // Fault status input pin (active high: HIGH = OK, LOW = FAULT)
};

struct EncoderPins
{
  uint8_t channelA; // Pulse output A (must support external interrupt)
  uint8_t channelB; // Phase-shifted GPIO B (must support external interrupt)
};

// Define motor driver pins according to your table
constexpr MotorPins MOTOR_PINS[2] = {
    {3, 4, 5, 30}, // Motor 1: M1 PWM -> D3, M1 DIR -> D4, M1 SLP -> D5, M1 FLT -> D30
    {6, 7, 8, 32}  // Motor 2: M2 PWM -> D6, M2 DIR -> D7, M2 SLP -> D8, M2 FLT -> D32
};

// Define encoder pins according to your table
constexpr EncoderPins ENCODER_PINS[2] = {
    {22, 24}, // Encoder 1: Channel 1A -> D22, Channel 1B -> D24
    {26, 28}  // Encoder 2: Channel 2A -> D26, Channel 2B -> D28
};

// ------------------------ Control parameters ------------------------
constexpr uint8_t PWM_MAX = 255;           // Arduino Mega's analogWrite is 8-bit (0-255)
constexpr uint16_t CONTROL_INTERVAL_US = 5000; // 5 ms control loop period
constexpr float CONTROL_PERIOD_S = CONTROL_INTERVAL_US / 1000000.0f;
constexpr uint32_t TELEMETRY_INTERVAL_MS = 50; // Original interval for *collecting* data

// Speed Data Buffer Configuration
constexpr uint8_t TELEMETRY_BUFFER_SIZE = 20; // Number of telemetry frames to buffer (as requested)
String telemetryBuffer[TELEMETRY_BUFFER_SIZE]; // Buffer to hold telemetry strings
uint8_t bufferIndex = 0;                       // Current index in the buffer
unsigned long lastBufferSendMs = 0;            // Time of last buffer send

// TODO: Update these values based on your actual hardware specs!
constexpr int32_t ENCODER_CPR = 2048;      // Encoder counts per revolution (placeholder)
constexpr float MAX_RPM = 4000.0f;         // Maximum mechanical RPM (placeholder)

enum class CommandMode
{
  INDEPENDENT,
  DIFFERENTIAL
};

struct CalibrationData
{
  float kp[2];          // Proportional gain for each motor
  float ki[2];          // Integral gain for each motor
  float kd[2];          // Derivative gain for each motor
  float feedForward[2]; // Feedforward coefficient for each motor
};

struct MotorState
{
  float targetRpm = 0.0f;    // Target RPM for this motor
  float actualRpm = 0.0f;    // Measured RPM from encoder
  float integral = 0.0f;     // Integral term for PID
  float lastError = 0.0f;    // Last error value for derivative calculation
  float controlEffort = 0.0f; // Normalized control effort (-1.0 to 1.0)
  bool enabled = true;       // Is this motor currently enabled?
};

CalibrationData calib;
MotorState motors[2];
CommandMode controlMode = CommandMode::INDEPENDENT;

volatile int32_t encoderCounts[2] = {0, 0}; // Volatile for safe access from ISR

bool driverHealthy[2] = {true, true}; // Track if each driver is healthy
unsigned long lastControlMicros = 0;
unsigned long lastTelemetryCollectMs = 0; // Changed name for clarity

// --------------- Speed profile sequence (optional) ------------------
struct ProfileStep
{
  float rpm[2];        // Target RPM for both motors
  uint32_t durationMs; // Duration of this step in milliseconds
};

ProfileStep profileQueue[8];
uint8_t profileHead = 0;
uint8_t profileTail = 0;
bool profileActive = false;
unsigned long profileStepMs = 0;

// --------------------- Utility / hardware helpers -------------------
inline void setMotorEnable(uint8_t motorIdx, bool enable)
{
  // Set the sleep/enable pin: Active High means HIGH = Enabled
  digitalWrite(MOTOR_PINS[motorIdx].sleep, enable ? HIGH : LOW);
  motors[motorIdx].enabled = enable;
}

inline void applyMotorPwm(uint8_t motorIdx, float effort)
{
  // Clamp effort to [-1.0, 1.0]
  effort = constrain(effort, -1.0f, 1.0f);

  // Determine direction: positive effort = forward, negative = reverse
  bool direction = effort >= 0.0f;

  // Convert normalized effort to 8-bit PWM duty cycle (0-255)
  uint8_t duty = static_cast<uint8_t>(fabsf(effort) * PWM_MAX);

  // Set direction pin
  digitalWrite(MOTOR_PINS[motorIdx].dir, direction ? HIGH : LOW);

  // Apply PWM signal to speed control pin
  analogWrite(MOTOR_PINS[motorIdx].pwm, duty);
}

inline int32_t readAndZeroEncoder(uint8_t motorIdx)
{
  noInterrupts(); // Disable interrupts to safely read shared variable
  int32_t count = encoderCounts[motorIdx];
  encoderCounts[motorIdx] = 0;
  interrupts();  // Re-enable interrupts
  return count;
}

float countsToRpm(int32_t counts)
{
  // Calculate revolutions per second
  float revolutions = static_cast<float>(counts) / ENCODER_CPR;
  float rps = revolutions / CONTROL_PERIOD_S;

  // Convert to RPM
  return rps * 60.0f;
}

void serviceQuadrature(uint8_t idx, uint8_t pinA, uint8_t pinB, bool channelA)
{
  // Read current state of encoder pins
  int a = digitalRead(pinA);
  int b = digitalRead(pinB);

  Serial.print("Encoder ");
  Serial.print(idx);
  Serial.print(": A=");
  Serial.print(a);
  Serial.print(", B=");
  Serial.println(b);

  

  // Increment or decrement counter based on phase relationship
  if (channelA)
  {
    encoderCounts[idx] += (a == b) ? 1 : -1;
  }
  else
  {
    encoderCounts[idx] += (a != b) ? 1 : -1;
  }
}

// Interrupt Service Routines for encoder channels
void encoder0A() { serviceQuadrature(0, ENCODER_PINS[0].channelA, ENCODER_PINS[0].channelB, true); }
void encoder0B() { serviceQuadrature(0, ENCODER_PINS[0].channelA, ENCODER_PINS[0].channelB, false); }
void encoder1A() { serviceQuadrature(1, ENCODER_PINS[1].channelA, ENCODER_PINS[1].channelB, true); } 
void encoder1B() { serviceQuadrature(1, ENCODER_PINS[1].channelA, ENCODER_PINS[1].channelB, false); }

// ------------------------ Config handling (EEPROM) ---------------------------
void loadCalibration()
{
  // Load calibration data from EEPROM starting at address 0
  EEPROM.get(0, calib);

  // If calibration data is uninitialized (all zeros), set defaults
  if (calib.kp[0] == 0 && calib.ki[0] == 0 && calib.kd[0] == 0)
  {
    calib = {
        {0.12f, 0.12f}, // Default KP for motor 1 and 2
        {0.45f, 0.45f}, // Default KI for motor 1 and 2
        {0.0008f, 0.0008f}, // Default KD for motor 1 and 2
        {0.00025f, 0.00025f}, // Default feedforward for motor 1 and 2
    };
    saveCalibration(); // Write default values to EEPROM
  }
}

void saveCalibration()
{
  // Save calibration data to EEPROM starting at address 0
  EEPROM.put(0, calib);
  // Note: On Arduino Mega, EEPROM.put() writes immediately, no need for commit()
}

// ----------------------- Safety handling (Driver faults only) ----------------------------
void disableAllMotors()
{
  // Stop all motors by setting PWM to 0 and disabling them
  for (uint8_t i = 0; i < 2; ++i)
  {
    applyMotorPwm(i, 0.0f);
    setMotorEnable(i, false);
  }
}

void checkSafetyInputs()
{
  // Check motor driver fault status (Active High: HIGH = OK, LOW = FAULT)
  // Note: Your table says "Active high (if low, not working)" so we invert logic for health
  driverHealthy[0] = digitalRead(MOTOR_PINS[0].fault) == HIGH; // HIGH = healthy
  driverHealthy[1] = digitalRead(MOTOR_PINS[1].fault) == HIGH; // HIGH = healthy

  // No global fault signal (NFault_PIN) defined, so only check individual drivers

  // If any driver is unhealthy, disable all motors
  if (!driverHealthy[0] || !driverHealthy[1])
  {
    disableAllMotors();
  }
  else
  {
    // Enable motors only if their respective drivers are healthy
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
    // Read encoder counts since last loop and reset counter
    int32_t delta = readAndZeroEncoder(i);

    // Convert encoder counts to RPM
    motors[i].actualRpm = countsToRpm(delta);

    // Calculate error between target and actual RPM
    float error = motors[i].targetRpm - motors[i].actualRpm;

    // Update integral term (anti-windup: clamp to prevent saturation)
    motors[i].integral += error * CONTROL_PERIOD_S;
    motors[i].integral = constrain(motors[i].integral, -MAX_RPM, MAX_RPM);

    // Calculate derivative term
    float derivative = (error - motors[i].lastError) / CONTROL_PERIOD_S;
    motors[i].lastError = error;

    // Calculate PID output
    float pid = calib.kp[i] * error + calib.ki[i] * motors[i].integral + calib.kd[i] * derivative;

    // Add feedforward component
    float ff = calib.feedForward[i] * motors[i].targetRpm;

    // Combine PID and FF, then normalize to [-1.0, 1.0]
    motors[i].controlEffort = constrain((pid + ff) / MAX_RPM, -1.0f, 1.0f);

    // Apply control effort only if motor is enabled (no fault)
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
  // Check if queue is full
  uint8_t next = (profileTail + 1) % 8;
  if (next == profileHead)
  {
    Serial.println(F("ERR,PROFILE_FULL"));
    return;
  }
  // Add new step to queue
  profileQueue[profileTail] = {{rpm0, rpm1}, duration};
  profileTail = next;
}

void handleProfile()
{
  // Start next profile step if queue is not empty and no active step
  if (!profileActive && profileHead != profileTail)
  {
    ProfileStep &step = profileQueue[profileHead];
    // Set target RPM for both motors, clamped to max/min limits
    motors[0].targetRpm = constrain(step.rpm[0], -MAX_RPM, MAX_RPM);
    motors[1].targetRpm = constrain(step.rpm[1], -MAX_RPM, MAX_RPM);
    profileActive = true;
    profileStepMs = millis(); // Record start time
  }
  else if (profileActive)
  {
    // Check if current step duration has elapsed
    ProfileStep &step = profileQueue[profileHead];
    if (millis() - profileStepMs >= step.durationMs)
    {
      // Move to next step
      profileHead = (profileHead + 1) % 8;
      profileActive = false;
    }
  }
}

void setTargetsFromCommand(float leftRpm, float rightRpm)
{
  if (controlMode == CommandMode::INDEPENDENT)
  {
    // Independent mode: set each motor to its own target RPM
    motors[0].targetRpm = constrain(leftRpm, -MAX_RPM, MAX_RPM);
    motors[1].targetRpm = constrain(rightRpm, -MAX_RPM, MAX_RPM);
  }
  else
  {
    // Differential mode: left = u + v, right = u - v
    float u = (leftRpm + rightRpm) * 0.5f; // Common velocity
    float v = (leftRpm - rightRpm) * 0.5f; // Differential velocity
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
    // Invalidate any queued profile steps when manual command is sent
    profileHead = profileTail;
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
    // Update calibration parameters
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
    saveCalibration(); // Persist changes to EEPROM
  }
  // Removed ESTOP and RESET commands as requested
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

// ------------------------ Telemetry Buffering --------------------------
void collectTelemetry() // Renamed from publishTelemetry
{
  // Format the telemetry string
  String telemetryLine = F("TEL,");
  telemetryLine += millis();
  telemetryLine += ',';
  telemetryLine += motors[0].targetRpm, 2;
  telemetryLine += ',';
  telemetryLine += motors[0].actualRpm, 2;
  telemetryLine += ',';
  telemetryLine += motors[1].targetRpm, 2;
  telemetryLine += ',';
  telemetryLine += motors[1].actualRpm, 2;
  // Removed force, current, temperature fields
  telemetryLine += ',';
  telemetryLine += driverHealthy[0];
  telemetryLine += ',';
  telemetryLine += driverHealthy[1];
  telemetryLine += ',';
  telemetryLine += false; // No E-Stop, always false
  telemetryLine += ',';
  telemetryLine += profileActive;
  telemetryLine += '\n'; // Add newline for proper parsing on PC side

  // Add the formatted string to the buffer
  if (bufferIndex < TELEMETRY_BUFFER_SIZE)
  {
    telemetryBuffer[bufferIndex] = telemetryLine;
    bufferIndex++;
  }
  // If buffer is full, it will overwrite the oldest entry implicitly
  // The send function will handle sending the current buffer contents
}

void sendTelemetryBuffer()
{
  if (bufferIndex > 0)
  {
    // Send all collected telemetry lines from the buffer at once
    for (uint8_t i = 0; i < bufferIndex; i++)
    {
      Serial.print(telemetryBuffer[i]);
    }
    Serial.flush(); // Wait for transmission to complete (optional, but good practice)
    bufferIndex = 0; // Reset buffer index after sending
  }
}

// ----------------------------- Setup --------------------------------
void configurePins()
{
  // Configure motor driver pins
  for (uint8_t i = 0; i < 2; ++i)
  {
    pinMode(MOTOR_PINS[i].dir, OUTPUT);
    pinMode(MOTOR_PINS[i].sleep, OUTPUT);
    pinMode(MOTOR_PINS[i].fault, INPUT_PULLUP); // Fault pin is input with pull-up
    setMotorEnable(i, true); // Initially enable motors
  }

  // Configure encoder pins (inputs)
  pinMode(ENCODER_PINS[0].channelA, INPUT);
  pinMode(ENCODER_PINS[0].channelB, INPUT);
  pinMode(ENCODER_PINS[1].channelA, INPUT);
  pinMode(ENCODER_PINS[1].channelB, INPUT);

  // Attach interrupts for encoder signals (CHANGE triggers on rising/falling edge)
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINS[0].channelA), encoder0A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINS[0].channelB), encoder0B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINS[1].channelA), encoder1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINS[1].channelB), encoder1B, CHANGE);
}

void setup()
{
  Serial.begin(115200); // Initialize serial communication
  loadCalibration();    // Load saved PID/FF parameters
  configurePins();      // Configure all I/O pins
  lastControlMicros = micros(); // Initialize timing variables
  lastTelemetryCollectMs = millis(); // Initialize timing for data collection
  lastBufferSendMs = millis();       // Initialize timing for buffer sending
  Serial.println(F("INFO,ARDUINO_MEGA_TREADMILL_READY_NO_ESTOP")); // Send ready message
}

// ------------------------------ Loop --------------------------------
void loop()
{
  unsigned long nowMicros = micros();
  // Run control loop every 5ms
  if (nowMicros - lastControlMicros >= CONTROL_INTERVAL_US)
  {
    lastControlMicros += CONTROL_INTERVAL_US;
    checkSafetyInputs(); // Check for driver faults
    if (true) // Always handle profile (since no E-Stop)
    {
      handleProfile();
    }
    runControl(); // Execute PID control for both motors
  }

  // Collect telemetry data at the original interval (e.g., 50ms)
  unsigned long nowMs = millis();
  if (nowMs - lastTelemetryCollectMs >= TELEMETRY_INTERVAL_MS)
  {
    lastTelemetryCollectMs = nowMs;
    collectTelemetry(); // Add a telemetry line to the buffer
  }

  // Send the buffer contents when it's full or a certain time has passed
  if (bufferIndex >= TELEMETRY_BUFFER_SIZE || (nowMs - lastBufferSendMs >= 1000)) // Send every 1000ms if not full
  {
    lastBufferSendMs = nowMs;
    sendTelemetryBuffer(); // Send all buffered data
  }

  pollSerial(); // Process incoming serial commands (non-blocking check)
}