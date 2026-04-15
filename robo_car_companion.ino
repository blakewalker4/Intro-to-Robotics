#include <basicMPU6050.h>
#include "BLEDevice.h"
#include "string.h"

// state declarations
enum MyState {
  IDLE,
  DANCE,
  SPIN,
  TURNLEFT,
  TURNRIGHT,
  TURNAROUND,
  COMEHERE,
  GOBACKWARD,
  GOFORWARD
};

MyState state = IDLE;

// Function definitions
void stop(int time_dur = 100);

// Motor and PWM stuff
/*
 * IN1   IN2   | Motor Behavior
 * ----------  | 
 * HIGH  LOW   | Clockwise (forward for right motor)
 * LOW   HIGH  | Counter-Clockwise (forward for left motor)
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
int max_adjustment = 50;  // Maximum speed change between motors

// IMU stuff
basicMPU6050<> imu;

// Bluetooth stuff
// The remote service we wish to connect to.
static BLEUUID serviceUUID("4fafc201-1fb5-459e-8fcc-c5c9c331914b");
// The characteristic of the remote service we are interested in.
static BLEUUID charUUID("beb5483e-36e1-4688-b7f5-ea07361b26a8");

static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
static BLERemoteCharacteristic* pRemoteCharacteristic;
static BLEAdvertisedDevice* myDevice;

static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
    Serial.print("data: ");

    // Copy into a null-terminated buffer before printing
    char buf[length + 1];
    memcpy(buf, pData, length);
    buf[length] = '\0';
    Serial.println(buf);

    if(state == IDLE){
      if(strcmp(buf, "Dance")==0){
        state = DANCE;
        dance();
      }
      else if(strcmp(buf, "Spin")==0){
        state = SPIN;
        spin(200);
      }
      else if(strcmp(buf, "TurnLeft")==0){
        state = TURNLEFT;
        rotateToAngleIMU(270, 190);
      }
      else if(strcmp(buf, "TurnRight")==0){
        state = TURNRIGHT;
        rotateToAngleIMU(90, 190);
      }
      else if(strcmp(buf, "TurnAround")==0){
        state = TURNAROUND;
        rotateToAngleIMU(180, 190);
      }
      else if(strcmp(buf, "ComeHere")==0){
        state = COMEHERE;
        //TODO
      }
      else if(strcmp(buf, "GoForward")==0){
        state = GOFORWARD;
        goStraightIMU(200);
      }
      else if(strcmp(buf, "GoBackward")==0){
        state = GOBACKWARD;
        goStraightIMU(200);
      }
    }
    else if (strcmp(buf, "Stop") == 0){
        pass;
      }
      // last class it could be is ComeHere
      // ComeHere is returned as "ComeHere:<doa>"
      else{
        state = COMEHERE
        uint8_t idx = 0;
        while(buf[idx] != ':'){
          idx++;
        }
        while(buf[idx] != '\0'){
          uint8_t idx2 = 0;
          come_here_buf[idx2++] = buf[idx++];
        }
        uint8_t doa_val = atoi(come_here_buff);
        // TODO add turn functionality
        // turn_to_angle(doa_val);
        Serial.println("doa val: %d", doa_val);
        // GoForward();
    }
    else{
      if(strcmp(buf, "Stop")==0){
        stop();
      }
    }
    
}

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
  }

  void onDisconnect(BLEClient* pclient) {
    connected = false;
    Serial.println("onDisconnect");
  }
};

bool connectToServer() {
  Serial.print("Forming a connection to ");
  Serial.println(myDevice->getAddress().toString().c_str());
  
  BLEClient*  pClient  = BLEDevice::createClient();
  Serial.println(" - Created client");

  pClient->setClientCallbacks(new MyClientCallback());

  // Connect to the remove BLE Server.
  pClient->connect(myDevice);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
  Serial.println(" - Connected to server");

  // Obtain a reference to the service we are after in the remote BLE server.
  BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr) {
    Serial.print("Failed to find our service UUID: ");
    Serial.println(serviceUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found our service");

  // Obtain a reference to the characteristic in the service of the remote BLE server.
  pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
  if (pRemoteCharacteristic == nullptr) {
    Serial.print("Failed to find our characteristic UUID: ");
    Serial.println(charUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found our characteristic");

  // Read the value of the characteristic.
  if(pRemoteCharacteristic->canRead()) {
    String value = pRemoteCharacteristic->readValue();
    Serial.print("The characteristic value was: ");
    Serial.println(value.c_str());
  }

  if(pRemoteCharacteristic->canNotify())
    pRemoteCharacteristic->registerForNotify(notifyCallback);

  connected = true;
  return true;
}

// Scan for BLE servers and find the first one that advertises the service we are looking for.
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
 // Called for each advertising BLE server.
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());

    // We have found a device, let us now see if it contains the service we are looking for.
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {

      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = true;

    } // Found our server
  } // onResult
}; // MyAdvertisedDeviceCallbacks

float deg2rad(float degrees){
  return degrees*3.141592/180;
}

void turnCW(int speed, int time_dur){
  // set left motors to reverse
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

void turnCCW(int speed, int time_dur){
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

void goForward(int speed){
  // set left motors to forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  // set right motors to forward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  ledcWrite(ENA, speed);
  ledcWrite(ENB, speed);
}

void goBackward(int speed){
  // set left motors to reverse
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  // set right motors to reverse
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  ledcWrite(ENA, speed);
  ledcWrite(ENB, speed);
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
  state = IDLE;
}

void spin(int speed){ // turn CW until another command comes in
  // set left motors to forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  // set right motors to reverse
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  ledcWrite(ENA, speed);
  ledcWrite(ENB, speed);
}

void dance(){
  goForward(200);
  delay(1000);
  goBackward(200);
  delay(1000);
  turnCW(200, 1000);
  turnCCW(200, 1000);
  stop();
}

void goStraightIMU(int base_speed){ // Unified function for goForward and goBackward
  // PID constants for drift correction
  float Kp_forward = 40;  // Proportional gain - adjust as neccessary
  float Ki_forward = 0.3; // Integral gain - adjust as neccessary
  float Kp_backward = 40;  // Proportional gain - adjust as neccessary
  float Ki_backward = 0.3; // Integral gain - adjust as neccessary
  float Kp;
  float Ki;
  float Kd = 0;  // Derivative gain - adjust as neccessary
  float previous_error = 0;
  float integral = 0;
  unsigned long last_time = micros();
  unsigned long start_time = micros();
  unsigned long now = micros();
  int left_speed;
  int right_speed;
  float correction;
  unsigned long timeout = 7000; // 7 second timeout safety
  while((state == GOFORWARD || state == GOBACKWARD) && (now-start_time < timeout*1000)){
    // Calculate loop time for proper PID
    now = micros();
    float dt = (now - last_time) / 1000000.0;  // Convert to seconds
    last_time = now;
    
    // Get gyro Z-axis (turning rate)
    float turn_rate = imu.gz();  // degrees per second
    
    // We want turn rate to be 0 (going straight)
    float error = 0 - turn_rate;
    
    // PID calculations
    integral += error * dt;
    float derivative = (error - previous_error) / dt;
    
    // Calculate motor speeds
    if(state == GOFORWARD){
      Kp = Kp_forward;
      Ki = Ki_forward;

      // Calculate correction
      correction = Kp * error + Ki * integral + Kd * derivative;
      
      // Constrain correction
      correction = constrain(correction, -max_adjustment, max_adjustment);
      
      // Set motor speeds
      left_speed = base_speed - correction;   // If correction positive, left slower
      right_speed = base_speed + correction;  // If correction positive, right faster
      
      // Set motor directions
      // set left motors to forward
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      // set right motors to forward
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
    }
    // NOTE: when going backward, the PID controller is doing very little 
    // to correct for the integral error, even if Ki_backward is very high, dont know why.
    // The controlled version is still slightly better though
    else if(state == GOBACKWARD){
      Kp = Kp_backward;
      Ki = Ki_backward;

      // Calculate correction
      correction = Kp * error + Ki * integral + Kd * derivative;
      
      // Constrain correction
      correction = constrain(correction, -max_adjustment, max_adjustment);
      
      // Set motor speeds
      left_speed = base_speed + correction;   // If correction positive, left slower
      right_speed = base_speed - correction;  // If correction positive, right faster
            
      // Set motor directions
      // set left motors to backward
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      // set right motors to backward
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
    }
    else{
      // INVALID STATE
      break;
    }
    // Constrain speeds
    left_speed = constrain(left_speed, 0, 255);
    right_speed = constrain(right_speed, 0, 255);
    
    // Apply PWM speeds
    ledcWrite(ENA, left_speed);
    ledcWrite(ENB, right_speed);
    
    // Debug output
    Serial.print("Turn Rate: ");
    Serial.println(turn_rate);
    Serial.print(" deg/s, Correction: ");
    Serial.println(correction);
    Serial.print(" L: ");
    Serial.println(left_speed);
    Serial.print(" R: ");
    Serial.println(right_speed);
    Serial.print("Time elapsed: ");
    Serial.println(float((now-start_time)/1000000));
    Serial.println("");
    
    previous_error = error;
    delay(10);  // 100Hz control loop
  }
  Serial.println("exiting loop");
  stop();
}

void rotateToAngleIMU(float target_angle_degrees, int base_speed) {
  // PID constants for rotation (tune these)
  float Kp_rotate = 2;  // Proportional gain
  float Ki_rotate = 0.05;  // Integral gain  
  float Kd_rotate = 0;   // Derivative gain
  float current_angle = 0;
  float previous_error = 0;
  float integral = 0;
  unsigned long last_time = micros();
  unsigned long start_time = millis();
  unsigned long timeout = 5000; // 5 second timeout safety

  bool clockwise;
  float target_rad;
  // For rotations > 180°, consider the shortest path
  if(target_angle_degrees <= 180) {
    // Turn clockwise for angles 0-180
    clockwise = true;
    target_rad = deg2rad(target_angle_degrees);
  } else {
    // Turn counter-clockwise for angles 181-360 (shorter)
    clockwise = false;
    target_rad = deg2rad(360 - target_angle_degrees);
  }
  
  // Rotate until reaching target angle
  while(state != IDLE && abs(current_angle) < target_rad) {
    unsigned long now = micros();
    float dt = (now - last_time) / 1000000.0;
    if(dt > 0.05) dt = 0.05;
    last_time = now;
    
    // Get current rotation rate
    float turn_rate = imu.gz();
    
    // Integrate to get angle
    current_angle += abs(turn_rate) * dt;
    
    // Calculate error (how much more to turn)
    float error = target_rad - abs(current_angle);
    
    // PID calculations
    integral += error * dt;
    
    float derivative = (error - previous_error) / dt;
    
    // Calculate motor speed based on error
    int speed = base_speed + Kp_rotate * error + Ki_rotate * integral + Kd_rotate * derivative;
    speed = constrain(speed, 170, 230);

    // Set motors for rotation
    if(clockwise) {
      // Clockwise: left forward, right backward
      digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
      digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);

    }
    else {
      // Counter-clockwise: left backward, right forward  
      digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
    }
    
    // Apply speed
    ledcWrite(ENA, speed);
    ledcWrite(ENB, speed);
    
    // Debug output
    Serial.print("Rotated: ");
    Serial.print(current_angle);
    Serial.print(" / ");
    Serial.print(target_angle_degrees);
    Serial.print(" | Rate: ");
    Serial.println(turn_rate);    
      
    // Timeout safety
    if(millis() - start_time > timeout) {
      Serial.println("Rotation timeout!");
      break;
    }
    
    delay(5);  // 200Hz update rate
  }
  
  // Stop motors
  stop();
  
  Serial.print("Final rotation: ");
  Serial.println(current_angle);
}

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
  // BLUETOOTH STUFF: COMMENTING OUT FOR TESTING
  /*
  Serial.println("Starting Arduino BLE Client application...");
  BLEDevice::init("");

  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 5 seconds.
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, false);
  */
  // set all motors to off by default
  stop();
}

void loop() {
  // TESTING PURPOSES
  /*
  state = SPIN;
  // Test 90° clockwise turn
  rotateToAngleIMU(90, 190);
  delay(2000);
  state = SPIN;
  // Test 90° counter-clockwise turn  
  rotateToAngleIMU(270, 190);
  delay(2000);
  state = SPIN;
  // Test 180° turn around
  rotateToAngleIMU(180, 190);
  delay(2000);
  */

  // Test clockwise command
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  ledcWrite(ENA, 200);
  ledcWrite(ENB, 200);
  delay(1000);
  stop();
  /*
  // If the flag "doConnect" is true then we have scanned for and found the desired
  // BLE Server with which we wish to connect.  Now we connect to it.  Once we are 
  // connected we set the connected flag to be true.
  if (doConnect == true) {
    if (connectToServer()) {
      Serial.println("We are now connected to the BLE Server.");
    } else {
      Serial.println("Failed to connect to server.");
    }
    doConnect = false;
  }

  // If we are connected to a peer BLE Server, update the characteristic each time we are reached
  // with the current time since boot.
  if (connected) {
    String newValue = "";
    
    // Set the characteristic's value to be the array of bytes that is actually a string.
    pRemoteCharacteristic->writeValue(newValue.c_str(), newValue.length());
  }else if(doScan){
    BLEDevice::getScan()->start(0);  // this is just example to start scan after disconnect
  }
  */
  delay(1000); // Delay a second between loops.
}