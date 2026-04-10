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
        turnCCW(200,1000);
      }
      else if(strcmp(buf, "TurnRight")==0){
        state = TURNRIGHT;
        turnCW(200,1000);
      }
      else if(strcmp(buf, "TurnAround")==0){
        state = TURNAROUND;
        turnCCW(200,1000); //ADD IMU CODE
      }
      else if(strcmp(buf, "ComeHere")==0){
        state = COMEHERE;
      }
      else if(strcmp(buf, "GoForward")==0){
        state = GOFORWARD;
        goForwardIMU(200);
      }
      else if(strcmp(buf, "GoBackward")==0){
        state = GOBACKWARD;
        goBackward(200);
      }
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

void goForwardIMU(int speed){
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
  /*
  Serial.print("Turn Rate: ");
  Serial.print(turn_rate);
  Serial.print(" deg/s, Correction: ");
  Serial.print(correction);
  Serial.print(" L: ");
  Serial.print(left_speed);
  Serial.print(" R: ");
  Serial.println(right_speed);
  */

  previous_error = error;
  delay(10);  // 100Hz control loop
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

  // set all motors to off by default
  stop();

  // setup bluetooth


  last_time = micros();
}

void loop() {
  // goForwardIMU();
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
  
  delay(1000); // Delay a second between loops.
}