/**************************************************************************************************
*                 UNIVERSITY OF BATH ELECTRONIC & ELECTRICAL ENGINEERING DEPARTMENT
*
* File: fan_control.ino - Software Version 1.0.0
* Device: Arduino Nano ESP32
* Created on: 16 September 2025, 13:46
* Last updated: 07 October 2025, 10:42
* Author: Richmond Afeawo
*
* Description: This code makes use of the fan drive interface boards as part of the Year 3 Fan
* Control lab in the EE Department.
* It makes use of a rotary encoder as an input to adjust the target speed of the fan.
* The built-in tachometer on the fan is used to measure the actual fan speed.
* There is no feedback in this system, so the PWM output remains fixed even when external factors
* require a change in the fan speed
*
* Notes: - 12V is supplied to the fan drive interface board
*        - 3V3 for the interface board is provided from the Arduino
**************************************************************************************************/
 
// Debug controls, uncomment to enable debug output
#define DEBUG
// import PID control
#include <PID_v1.h>

// control vars
double rpm, rpm_pwm, setpoint, output;
// PID Tunings — start with these and tune later
double Kp = 2.5;
double Ki = 0.0;
double Kd = 0.0;
// PID internal state
double pidIntegral = 0;
double pidPrevError = 0;
unsigned long pidPrevTime = 0;

// PWM output and fan tacho input pins
const int PWM_OUT = 2;
const int TACHO_IN = 3;

// Rotary encoder input pins
const int ROT_IN_1 = 4;
const int ROT_IN_2 = 5;

// Measured maximum and minimum fan speeds
const long MIN_RPM = 660;
const long MAX_RPM = 1940;

// Maximum and minimum period from fan's tachometer
const long MIN_PERIOD = 14500;
const long MAX_PERIOD = 43000;

// Rotary encoder variables for setup
volatile long encoderPos = 1;   // Keep track of encoder value
volatile int lastEncoded = 0;   // Track last change in encoder input waveform
const long ENCODER_MIN = 1;     // Minimum limit of encoder value
const long ENCODER_MAX = 255;   // Maximum value of encoder value

// Variable to store the measured period of the tachometer signal from the fan
unsigned long period = 0;

int state = 0;

void setup() {
  // Initialize serial comms:
  Serial.begin(9600);

  // Set pin properties
  pinMode(PWM_OUT, OUTPUT);
  pinMode(TACHO_IN, INPUT_PULLUP);
  pinMode(ROT_IN_1, INPUT_PULLUP);
  pinMode(ROT_IN_2, INPUT_PULLUP);

  // Interrupts for encoder pins (Pin Change Interrupts)
  attachInterrupt(digitalPinToInterrupt(ROT_IN_1), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ROT_IN_2), updateEncoder, CHANGE);
  
  // Start fan off at initial speed
  analogWrite(PWM_OUT, 1);
  delay(500); // Delay to allow fan time to start up
}

// Main code to run continuously
void loop() {
  // Measure fan speed from tachometer
  checkPeriod();

  // Convert tachometer period to RPM
  const int PULSES_PER_REV = 2;
  double rpm = 0;
  if (period > 0) rpm = 60000000.0 / (period * PULSES_PER_REV);
  //input = rpm;

  // Update PID setpoint from encoder
  noInterrupts();
  setpoint = encoderPos;
  interrupts();

  // Convert setpoint to RMP
  rpm_pwm = map(rpm, MIN_RPM, MAX_RPM, 1, 255);

  // Compute PID
  double pidOut = computePID(setpoint, rpm_pwm);

  // Constrain to PWM range
  pidOut = constrain(pidOut, 0, 255);
  output = pidOut;
  
  static unsigned long lastPrint = 0; // Keep track of when data was last printed to Serial

  // Update outputs every 100ms
  if (millis() - lastPrint >= 100) {
#ifdef DEBUG
    Serial.print("Setpoint:"); Serial.print(setpoint);
    Serial.print(",Output:");  Serial.print(output);
    //Serial.print(",Period:");  Serial.print(period);
    Serial.print(",RPM:");  Serial.print(rpm);
    Serial.print(",RPM_PWM:");  Serial.println(rpm_pwm);
#endif   
    lastPrint = millis();
 
    // Update PWM Output
    analogWrite(PWM_OUT, output);
  }
}

// FUNCTIONS
void checkPeriod(){
  // Variables to track history of tachometer measurements
  static unsigned long lastEdgeTime = 0;
  static unsigned long lastValidTime = 0;
  
  // Acquire current tachometer state and time
  state = digitalRead(TACHO_IN);
  unsigned long now = micros();
  //Serial.print(",State:");  Serial.print(state);

  // Detect falling edge
  if (state == LOW) {
    unsigned long diff = now - lastEdgeTime;
    lastEdgeTime = now;
    //Serial.println("Tach Edge!");

    // Debounce: ignore edges that come too quickly
    if (diff > 2000) {  
      period = now - lastValidTime;
      lastValidTime = now;
    }
  }
}

void updateEncoder() {
  int MSB = digitalRead(ROT_IN_1);
  int LSB = digitalRead(ROT_IN_2);

  int encoded = (MSB << 1) | LSB; // Bit-field storing current state of rotary encoder pins
  int sum = (lastEncoded << 2) | encoded; // Combined bit-field holds previous and new pin values from rotary encoder
  // Gray code decoding
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderPos--;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderPos++;

  // Constrain encoder values between 1 and 255
  encoderPos = constrain(encoderPos, ENCODER_MIN, ENCODER_MAX);

  lastEncoded = encoded;
}

double computePID(double setpoint, double measurement) {
  unsigned long now = micros();
  double dt = (now - pidPrevTime) / 1e6;  // convert µs → seconds
  pidPrevTime = now;

  if (dt <= 0) dt = 1e-3;   // safety

  // Error terms
  double error = setpoint - measurement;

  // Integral
  pidIntegral += error * dt;

  // Derivative
  double derivative = (error - pidPrevError) / dt;
  pidPrevError = error;

  // PID output
  double output = setpoint + (Kp * error) + (Ki * pidIntegral) + (Kd * derivative);

  return output;
}