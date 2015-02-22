#include <Servo.h>

#define AILERON 3
#define ELEVATOR 4
#define RUDDER 7
#define THROTTLE 6

#define M1 14
#define M2 15
#define M3 16
#define M4 17

Servo motor[4];

void setup() {
  // initialize input pins
  pinMode(AILERON, INPUT);
  pinMode(ELEVATOR, INPUT);
  pinMode(RUDDER, INPUT);
  pinMode(THROTTLE, INPUT);
  
  // attach motor to pins
  motor[0].attach(M1);
  motor[1].attach(M2);
  motor[2].attach(M3);
  motor[3].attach(M4);
  
  // turn off all motors
  motor[0].writeMicroseconds(1000);
  motor[1].writeMicroseconds(1000);
  motor[2].writeMicroseconds(1000);
  motor[3].writeMicroseconds(1000);
 
  // initialize serial comm
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  int duration[4];
  duration[0] = pulseIn(AILERON, HIGH);
  duration[1] = pulseIn(ELEVATOR, HIGH);
  duration[2] = pulseIn(RUDDER, HIGH);
  duration[3] = pulseIn(THROTTLE, HIGH);
  
  Serial.print("A/E/R/T = ");
  Serial.print(duration[0]);
  Serial.print("\t");
  Serial.print(duration[1]);
  Serial.print("\t");
  Serial.print(duration[2]);
  Serial.print("\t");
  Serial.println(duration[3]);
  
  motor[0].writeMicroseconds(duration[3]);
  motor[1].writeMicroseconds(duration[3]);
  motor[2].writeMicroseconds(duration[3]);
  motor[3].writeMicroseconds(duration[3]);
  
}
