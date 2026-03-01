#include <basicMPU6050.h>

// Motor and PWM stuff
/*
 * IN1   IN2   | Motor Behavior
 * ----------  | 
 * HIGH  LOW   | Forward
 * LOW   HIGH  | Backward
 * HIGH  HIGH  | Brake (fast stop)
 * LOW   LOW   | Coast (slow stop)
 */
// Note: Motor speed is a 8 bit value between 0 and 255
//       but anything below about 160 will not make the motors move


// left motors
#define ENA 32 // PWM pin for left motor speed
#define IN1 33 // Digital pin - direction
#define IN2 25 // Digital pin - direction

// right motors
#define ENB 14 // PWM pin for right motor speed
#define IN3 26 // Digital pin - direction
#define IN4 27 // Digital pin - direction

// PWM settings
#define PWM_FREQ 5000      // 5 kHz is good for DC motors
#define PWM_RESOLUTION 8   // 8-bit resolution (0-255)

// Speed settings
int base_speed = 170;  // (0-255)
int max_adjustment = 50;  // Maximum speed change between motors


// IMU stuff
basicMPU6050<> imu;

// PID constants for drift correction
float Kp = 1.5;  // Proportional gain - adjust as neccessary
float Ki = 0.05; // Integral gain - adjust as neccessary
float Kd = 0.1;  // Derivative gain - adjust as neccessary

float previous_error = 0;
float integral = 0;
unsigned long last_time = 0;


void setup() {
  // Setup PWM for speed control
  ledcAttach(ENA, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(ENB, PWM_FREQ, PWM_RESOLUTION);

  // Setup direction pins as regular digital outputs
  pinMode(IN1, OUTPUT); 
  pinMode(IN2, OUTPUT); 
  pinMode(IN3, OUTPUT); 
  pinMode(IN4, OUTPUT); 

  // Setup MPU6050
  imu.setup();
  imu.setBias();  // Calibrate gyro when car is stationary
  
  Serial.begin(115200);
  last_time = micros();
}

void loop() {
  // set all motors to off by default
  stop(100);
  
  // Calculate loop time for proper PID
  unsigned long now = micros();
  float dt = (now - last_time) / 1000000.0;  // Convert to seconds
  last_time = now;
  
  // Update gyro readings
  imu.updateBias();
  
  // Get gyro Z-axis (turning rate)
  float turn_rate = imu.gz();  // degrees per second
  
  // We want turnRate to be 0 (going straight)
  float error = 0 - turn_rate;
  
  // PID calculations
  integral += error * dt;
  float derivative = (error - previous_error) / dt;
  
  // Calculate correction
  float correction = Kp * error + Ki * integral + Kd * derivative;
  
  // Constrain correction
  correction = constrain(correction, -max_adjustment, max_adjustment);
  
  // Calculate motor speeds
  int left_speed = base_speed - correction;   // If correction positive, left slower
  int right_speed = base_speed + correction;  // If correction positive, right faster
  
  // Constrain speeds
  left_speed = constrain(left_speed, 0, 255);
  right_speed = constrain(right_speed, 0, 255);

  // Set motor directions
  // set left motors to forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  // set right motors to forward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  
  // Apply PWM speeds
  ledcWrite(ENA, left_speed);
  ledcWrite(ENB, right_speed);
  
  // Debug output
  Serial.print("Turn Rate: ");
  Serial.print(turn_rate);
  Serial.print(" deg/s, Correction: ");
  Serial.print(correction);
  Serial.print(" L: ");
  Serial.print(left_speed);
  Serial.print(" R: ");
  Serial.println(right_speed);
  
  previous_error = error;
  delay(10);  // 100Hz control loop
  
}


void turnCCW(int speed, int time_dur){
  // set right motors to reverse
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  // set right motors to forward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  unsigned long start_time = millis();
  unsigned long cur_time = millis();
  while(cur_time - start_time < time_dur){
    ledcWrite(ENA, speed);
    ledcWrite(ENB, speed);
    cur_time = millis();
  }
}

void turnCW(int speed, int time_dur){
  // set left motors to forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  // set right motors to reverse
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  unsigned long start_time = millis();
  unsigned long cur_time = millis();
  while(cur_time - start_time < time_dur){
    ledcWrite(ENA, speed);
    ledcWrite(ENB, speed);
    cur_time = millis();
  }
}

void goForward(int speed, int time_dur){
  // set left motors to forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  // set right motors to forward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  unsigned long start_time = millis();
  unsigned long cur_time = millis();
  while(cur_time - start_time < time_dur){
    ledcWrite(ENA, speed);
    ledcWrite(ENB, speed);
    cur_time = millis();
  }
}

void goBackward(int speed, int time_dur){
  // set left motors to reverse
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  // set right motors to reverse
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  unsigned long start_time = millis();
  unsigned long cur_time = millis();
  while(cur_time - start_time < time_dur){
    ledcWrite(ENA, speed);
    ledcWrite(ENB, speed);
    cur_time = millis();
  }
}

void stop(int time_dur){
  // set left motors to stop
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  // set right motors to stop
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);
  ledcWrite(ENA, 255);
  ledcWrite(ENB, 255);
  delay(time_dur);
}

void testRun(int speed){
  turnCW(speed,1000);
  stop(100);
  turnCCW(speed,1000);
  stop(100);
  goForward(speed,1000);
  stop(100);
  goBackward(speed,1000);
  stop(100);
}

void speedSweep(int speed){
  goForward(speed,300);
  stop(100);
  goBackward(speed,300);
  stop(100);
}
