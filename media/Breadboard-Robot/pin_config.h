#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

#include <Arduino.h>

// ============================================================================
// HARDWARE PIN DEFINITIONS (Arduino Uno / Nano ATmega328P)
// ============================================================================

// Left Motor - L298N H-Bridge Direction & PWM
#define PIN_MOTOR_LEFT_PWM      5   // Timer0 PWM Output
#define PIN_MOTOR_LEFT_IN1      7   // Digital Output
#define PIN_MOTOR_LEFT_IN2      8   // Digital Output

// Right Motor - L298N H-Bridge Direction & PWM
#define PIN_MOTOR_RIGHT_PWM     6   // Timer0 PWM Output
#define PIN_MOTOR_RIGHT_IN3     9   // Digital Output
#define PIN_MOTOR_RIGHT_IN4     10  // Digital Output

// Optical & Distance Sensors
#define PIN_IR_LEFT_SENSOR      2   // Digital Input (Active LOW)
#define PIN_IR_RIGHT_SENSOR     3   // Digital Input (Active LOW)
#define PIN_ULTRASONIC_TRIG     11  // 10us Trigger Pulse
#define PIN_ULTRASONIC_ECHO     12  // Echo Pulse Input

// Debug Status Indicator
#define PIN_STATUS_LED          13  // Onboard LED

// ============================================================================
// CALIBRATION & THRESHOLDS
// ============================================================================
#define DEFAULT_MOTOR_SPEED     185   // 0-255 PWM duty cycle (~72%)
#define OBSTACLE_THRESHOLD_CM   15.0  // Distance to trigger avoidance maneuver

#endif // PIN_CONFIG_H
