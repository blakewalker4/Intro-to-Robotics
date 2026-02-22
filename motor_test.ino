/*
* Intro to robotics lab project
* Robot car motor control code
*/ 

// left motors
#define ENA 32 // PWM pin for left motor speed
#define IN1 33 // Digital pin - left motor behavior 1
#define IN2 25 // Digital pin - left motor behavior 2

// right motors
#define ENB 14 // PWM pin for right motor speed
#define IN3 26 // Digital pin - right motor behavior 1
#define IN4 27 // Digital pin - right motor behavior 2

#define ON 21

/*
 * IN1/3   IN2/4   | Motor Behavior
 * ----------------| 
 * HIGH    LOW     | Forward
 * LOW     HIGH    | Backward
 * HIGH    HIGH    | Brake (fast stop)
 * LOW     LOW     | Coast (slow stop)
 */

// Note: Motor speed is a 8 bit value between 0 and 255
//       but anything below about 160 will not make the motors move

// PWM settings
#define PWM_FREQ 5000      // 5 kHz is good for DC motors
#define PWM_RESOLUTION 8   // 8-bit resolution (0-255)

void setup() {
  // Setup PWM for speed control
  ledcAttach(ENA, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(ENB, PWM_FREQ, PWM_RESOLUTION);

  // Setup direction pins as regular digital outputs
  pinMode(IN1, OUTPUT); 
  pinMode(IN2, OUTPUT); 
  pinMode(IN3, OUTPUT); 
  pinMode(IN4, OUTPUT); 
  
  pinMode(ON, INPUT);

  delay(100);
  Serial.begin(9600);
  
}

void loop() {
  bool on_state = digitalRead(ON);

  // set all motors to off by default
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);

  int speed = 170; // minimum 170
  
  while(on_state){
    speedSweep(speed); 
    if(255 > speed+5)
      speed += 5;
    else
      speed = 170;
    on_state = digitalRead(ON);
  }
  
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


