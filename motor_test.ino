// left motors
#define ENA 32 // PWM pin for left motor speed
#define IN1 33 // Digital pin - direction
#define IN2 25 // Digital pin - direction

// right motors
#define ENB 14 // PWM pin for right motor speed
#define IN3 26 // Digital pin - direction
#define IN4 27 // Digital pin - direction

#define ON 21

/*
 * IN1   IN2   | Motor Behavior
 * ----------  | 
 * HIGH  LOW   | Forward
 * LOW   HIGH  | Backward
 * HIGH  HIGH  | Brake (fast stop)
 * LOW   LOW   | Coast (slow stop)
 */

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
  Serial.println(on_state);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  
  while(on_state){
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    ledcWrite(ENA, 255);
    on_state = digitalRead(ON);
    delay(100);
  }
  
  
}
