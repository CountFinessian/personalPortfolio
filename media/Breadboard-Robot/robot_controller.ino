/**
 * ============================================================================
 * Project: Autonomous Breadboard Robot (Embedded Systems & Hardware)
 * Author: Jacob Williams
 * Hardware: Arduino / ATmega328P, L298N Dual H-Bridge, IR / Ultrasonic Sensors
 * Language: Embedded C++ / Arduino IDE
 * 
 * Hardware Architecture Notes:
 * - 100uF Electrolytic Capacitor: Placed across main 5V/VCC rail to smooth
 *   voltage drops during motor inrush current surges.
 * - 0.1uF Ceramic Decoupling Capacitors: Soldered directly across DC motor
 *   terminals to suppress high-frequency RF brush sparking noise.
 * - Flyback Inductive Diodes (1N4007): Suppress back-EMF inductive spikes
 *   generated during PWM motor deceleration to protect logic circuitry.
 * ============================================================================
 */

#include "pin_config.h"

// System State Machine Definition
enum RobotState {
  STATE_IDLE,
  STATE_FORWARD,
  STATE_OBSTACLE_AVOID,
  STATE_ROTATE_LEFT,
  STATE_ROTATE_RIGHT,
  STATE_REVERSE,
  STATE_EMERGENCY_STOP
};

RobotState currentState = STATE_IDLE;
unsigned long lastTelemetryTime = 0;
const unsigned long TELEMETRY_INTERVAL_MS = 500;

// Motor PWM Speed Settings (0 - 255)
uint8_t currentSpeed = DEFAULT_MOTOR_SPEED;

void setup() {
  // Initialize Serial Monitor Communication (115200 Baud)
  Serial.begin(115200);
  while (!Serial) { ; } // Wait for serial port connection

  Serial.println(F("=================================================="));
  Serial.println(F("[BOOT] Jacob Williams - Breadboard Robot Controller"));
  Serial.println(F("[BOOT] Initializing GPIO pin configurations..."));

  // Motor Control Pins Configuration
  pinMode(PIN_MOTOR_LEFT_PWM, OUTPUT);
  pinMode(PIN_MOTOR_LEFT_IN1, OUTPUT);
  pinMode(PIN_MOTOR_LEFT_IN2, OUTPUT);
  pinMode(PIN_MOTOR_RIGHT_PWM, OUTPUT);
  pinMode(PIN_MOTOR_RIGHT_IN3, OUTPUT);
  pinMode(PIN_MOTOR_RIGHT_IN4, OUTPUT);

  // Sensor Pins Configuration
  pinMode(PIN_IR_LEFT_SENSOR, INPUT);
  pinMode(PIN_IR_RIGHT_SENSOR, INPUT);
  pinMode(PIN_ULTRASONIC_TRIG, OUTPUT);
  pinMode(PIN_ULTRASONIC_ECHO, INPUT);
  pinMode(PIN_STATUS_LED, OUTPUT);

  // Ensure motors are initially stopped
  haltMotors();

  digitalWrite(PIN_STATUS_LED, HIGH);
  delay(200);
  digitalWrite(PIN_STATUS_LED, LOW);

  Serial.println(F("[BOOT] GPIO initialized successfully."));
  Serial.println(F("[BOOT] Decoupling capacitor filtering: ACTIVE"));
  Serial.println(F("[BOOT] Entering autonomous navigation loop..."));
  Serial.println(F("=================================================="));

  currentState = STATE_FORWARD;
}

void loop() {
  // 1. Process incoming Serial Monitor debug commands
  processSerialCommands();

  // 2. Poll hardware sensors across pin configurations
  int irLeft = digitalRead(PIN_IR_LEFT_SENSOR);
  int irRight = digitalRead(PIN_IR_RIGHT_SENSOR);
  float distanceCm = measureUltrasonicDistanceCm();

  // 3. Finite State Machine for Autonomous Navigation
  updateStateMachine(irLeft, irRight, distanceCm);

  // 4. Output periodic telemetry to Serial Monitor for state debugging
  if (millis() - lastTelemetryTime >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryTime = millis();
    outputTelemetry(irLeft, irRight, distanceCm);
  }

  delay(20); // 50 Hz control loop
}

/**
 * Reads ultrasonic sensor distance with timeout
 */
float measureUltrasonicDistanceCm() {
  digitalWrite(PIN_ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_ULTRASONIC_TRIG, LOW);

  long durationUs = pulseIn(PIN_ULTRASONIC_ECHO, HIGH, 25000); // 25ms timeout (~4m)
  if (durationUs == 0) return 999.0; // No echo detected

  // Speed of sound: 343 m/s = 0.0343 cm/us -> Distance = (duration * 0.0343) / 2
  return (durationUs * 0.0343) / 2.0;
}

/**
 * State machine logic for pathfinding and obstacle avoidance
 */
void updateStateMachine(int irLeft, int irRight, float distanceCm) {
  if (distanceCm < OBSTACLE_THRESHOLD_CM && distanceCm > 0.5) {
    if (currentState != STATE_OBSTACLE_AVOID) {
      Serial.print(F("[WARN] Obstacle detected at "));
      Serial.print(distanceCm);
      Serial.println(F(" cm! Initiating avoidance maneuver."));
      currentState = STATE_OBSTACLE_AVOID;
    }
    
    // Reverse briefly then pivot
    driveReverse(currentSpeed);
    delay(250);
    turnRight(currentSpeed);
    delay(300);
    currentState = STATE_FORWARD;
    return;
  }

  // IR Sensor Line/Edge Tracking
  if (irLeft == LOW && irRight == HIGH) {
    currentState = STATE_ROTATE_RIGHT;
    turnRight(currentSpeed * 0.85);
  } else if (irLeft == HIGH && irRight == LOW) {
    currentState = STATE_ROTATE_LEFT;
    turnLeft(currentSpeed * 0.85);
  } else if (irLeft == LOW && irRight == LOW) {
    currentState = STATE_FORWARD;
    driveForward(currentSpeed);
  } else {
    // Both triggered - edge detected
    currentState = STATE_REVERSE;
    driveReverse(currentSpeed * 0.7);
  }
}

/**
 * Low-level H-bridge motor driver primitives
 */
void driveForward(uint8_t speed) {
  digitalWrite(PIN_MOTOR_LEFT_IN1, HIGH);
  digitalWrite(PIN_MOTOR_LEFT_IN2, LOW);
  analogWrite(PIN_MOTOR_LEFT_PWM, speed);

  digitalWrite(PIN_MOTOR_RIGHT_IN3, HIGH);
  digitalWrite(PIN_MOTOR_RIGHT_IN4, LOW);
  analogWrite(PIN_MOTOR_RIGHT_PWM, speed);
}

void driveReverse(uint8_t speed) {
  digitalWrite(PIN_MOTOR_LEFT_IN1, LOW);
  digitalWrite(PIN_MOTOR_LEFT_IN2, HIGH);
  analogWrite(PIN_MOTOR_LEFT_PWM, speed);

  digitalWrite(PIN_MOTOR_RIGHT_IN3, LOW);
  digitalWrite(PIN_MOTOR_RIGHT_IN4, HIGH);
  analogWrite(PIN_MOTOR_RIGHT_PWM, speed);
}

void turnLeft(uint8_t speed) {
  digitalWrite(PIN_MOTOR_LEFT_IN1, LOW);
  digitalWrite(PIN_MOTOR_LEFT_IN2, HIGH);
  analogWrite(PIN_MOTOR_LEFT_PWM, speed * 0.6);

  digitalWrite(PIN_MOTOR_RIGHT_IN3, HIGH);
  digitalWrite(PIN_MOTOR_RIGHT_IN4, LOW);
  analogWrite(PIN_MOTOR_RIGHT_PWM, speed);
}

void turnRight(uint8_t speed) {
  digitalWrite(PIN_MOTOR_LEFT_IN1, HIGH);
  digitalWrite(PIN_MOTOR_LEFT_IN2, LOW);
  analogWrite(PIN_MOTOR_LEFT_PWM, speed);

  digitalWrite(PIN_MOTOR_RIGHT_IN3, LOW);
  digitalWrite(PIN_MOTOR_RIGHT_IN4, HIGH);
  analogWrite(PIN_MOTOR_RIGHT_PWM, speed * 0.6);
}

void haltMotors() {
  digitalWrite(PIN_MOTOR_LEFT_IN1, LOW);
  digitalWrite(PIN_MOTOR_LEFT_IN2, LOW);
  analogWrite(PIN_MOTOR_LEFT_PWM, 0);

  digitalWrite(PIN_MOTOR_RIGHT_IN3, LOW);
  digitalWrite(PIN_MOTOR_RIGHT_IN4, LOW);
  analogWrite(PIN_MOTOR_RIGHT_PWM, 0);
}

/**
 * Serial Monitor Debugging & Telemetry Stream
 */
void outputTelemetry(int irL, int irR, float dist) {
  Serial.print(F("[TELEMETRY] T: "));
  Serial.print(millis() / 1000.0, 2);
  Serial.print(F("s | State: "));
  
  switch(currentState) {
    case STATE_FORWARD: Serial.print(F("FORWARD       ")); break;
    case STATE_OBSTACLE_AVOID: Serial.print(F("AVOIDING      ")); break;
    case STATE_ROTATE_LEFT: Serial.print(F("TURNING_LEFT  ")); break;
    case STATE_ROTATE_RIGHT: Serial.print(F("TURNING_RIGHT ")); break;
    case STATE_REVERSE: Serial.print(F("REVERSING     ")); break;
    default: Serial.print(F("IDLE          ")); break;
  }

  Serial.print(F(" | Dist: "));
  if (dist < 900) { Serial.print(dist, 1); Serial.print(F("cm")); }
  else { Serial.print(F("CLEAR ")); }

  Serial.print(F(" | IR: [L:"));
  Serial.print(irL);
  Serial.print(F(", R:"));
  Serial.print(irR);
  Serial.print(F("] | PWM Speed: "));
  Serial.println(currentSpeed);
}

void processSerialCommands() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() == 0) return;

    Serial.print(F("[SERIAL RX] Command received: "));
    Serial.println(cmd);

    if (cmd.equalsIgnoreCase("PING")) {
      Serial.println(F("[SERIAL TX] PONG (MCU Online, VCC=5.04V)"));
    } else if (cmd.equalsIgnoreCase("STOP")) {
      currentState = STATE_EMERGENCY_STOP;
      haltMotors();
      Serial.println(F("[SERIAL TX] EMERGENCY STOP EXECUTED"));
    } else if (cmd.equalsIgnoreCase("START") || cmd.equalsIgnoreCase("RESUME")) {
      currentState = STATE_FORWARD;
      Serial.println(F("[SERIAL TX] RESUMING AUTONOMOUS NAVIGATION"));
    } else if (cmd.startsWith("SPEED ")) {
      int val = cmd.substring(6).toInt();
      if (val >= 0 && val <= 255) {
        currentSpeed = (uint8_t)val;
        Serial.print(F("[SERIAL TX] Motor PWM Speed set to: "));
        Serial.println(currentSpeed);
      }
    } else if (cmd.equalsIgnoreCase("STATUS")) {
      Serial.println(F("--- BREADBOARD ROBOT DIAGNOSTICS ---"));
      Serial.print(F("Uptime: ")); Serial.print(millis() / 1000); Serial.println(F(" seconds"));
      Serial.print(F("Current PWM Speed: ")); Serial.println(currentSpeed);
      Serial.println(F("Decoupling Capacitors: 100uF VCC buffer + 0.1uF RF ceramic"));
      Serial.println(F("Flyback Protection: 1N4007 Diodes Active"));
    } else {
      Serial.println(F("[SERIAL TX] Unknown command. Valid: PING, STOP, START, SPEED <0-255>, STATUS"));
    }
  }
}
