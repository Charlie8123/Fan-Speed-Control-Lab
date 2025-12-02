/**************************************************************************************************
*                 UNIVERSITY OF BATH ELECTRONIC & ELECTRICAL ENGINEERING DEPARTMENT
*
* Group 15 - Fan Control Lab
*
* Notes: - 12V is supplied to the fan drive interface board
*        - 3V3 for the interface board is provided from the Arduino
**************************************************************************************************/
 
// Debug controls, uncomment to enable debug output
#define DEBUG
// import PID control
#include <PID_v1.h>

// control vars
double rpm, rpm_pwm, setpoint, output, fan_voltage;
// PID Tunings — start with these and tune later
double Kp = 2.0;
double Ki = 0.0;
double Kd = 0;
// PID internal state
double pidIntegral = 0;
double pidPrevError = 0;
unsigned long pidPrevTime = 0;

double pidOut = 0;
unsigned long lastPID = 0;

// PWM output and fan tacho input pins
const int PWM_OUT = 2;
const int TACHO_IN = 3;

// Rotary encoder input pins
const int ROT_IN_1 = 4;
const int ROT_IN_2 = 5;

// Measured maximum and minimum fan speeds
const long MIN_RPM = 660;
const long MAX_RPM = 1940;
const double MAX_VOLTAGE = 12000;

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

  period = 0;

  Serial.println("t,setpoint_rpm,rpm,output_pwm,fan_voltage");

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
  //delay(500); // Delay to allow fan time to start up
}

// Main code to run continuously
void loop() {
  // Measure fan speed

  // Make exp curve to represent the nonlinear relationship between RMP and PWM
  checkPeriod();

  // Convert tachometer period to RPM
  const int PULSES_PER_REV = 2;
  double rpm = 0;
  if (period > 0) rpm = 60000000.0 / (period * PULSES_PER_REV);

  // ----- STEP INPUT -----
  // Step occurs at 10000 ms
  unsigned long t = millis();
  if (t < 10000) setpoint = 0;     // initial level (pre-step)
  if (t < 10000) rpm = 0;     // initial level (pre-step)
  else setpoint = 155;             // post-step target
  // -----------------------

  // Convert measurement to PWM scale used by your PID
  double setpoint_rpm = PWMtoRPM(setpoint);

  // Compute PID output
  const unsigned long PID_INTERVAL = 10000; // 10 ms

  if (micros() - lastPID >= PID_INTERVAL) {
      lastPID += PID_INTERVAL;
      pidOut = computePID(setpoint_rpm, rpm);
  }
  //Serial.print(setpoint_rpm); Serial.print(",");
  //Serial.println(pidOut);

  output = setpoint_rpm + pidOut;
  output = RPMtoPWM(output);
  // Compute equivalent fan voltage (mV)
  fan_voltage = MAX_VOLTAGE * (output / 255.0);

  static unsigned long lastPrint = 0;

  // Print every 100 ms
  if (t - lastPrint >= 100) {
#ifdef DEBUG
    Serial.print(t);       Serial.print(",");
    Serial.print(setpoint_rpm); Serial.print(",");
    Serial.print(rpm);      Serial.print(",");
    Serial.print(output);   Serial.print(",");
    Serial.println(fan_voltage);
    // Serial.print(rpm);       Serial.print(",");
    // Serial.print(setpoint_rpm);       Serial.print(",");
    // Serial.print(pidOut); Serial.print(",");
    // Serial.print(output); Serial.println(",");

#endif

    lastPrint = t;

    // Update fan output
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
    //Serial.println("Edge!");
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

float PWMtoRPM(int pwm) {
    const float a = 3.26e-3f;
    const float C = 0.59f;
    pwm = constrain(pwm, 1, 255);  // avoid log(0)
    return constrain(log(pwm / C) / a, 0, MAX_RPM);
}

int RPMtoPWM(float rpm) {
    const float a = 3.26e-3f;
    const float C = 0.59f;
    // Model: PWM = C * exp(a * rpm)
    float pwm = C * exp(a * rpm);
    // Limit range
    pwm = constrain(pwm, 0.0f, 255.0f);
    return (int)pwm;
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
  double output = (Kp * error) + (Ki * pidIntegral) + (Kd * derivative);

  return output;
}