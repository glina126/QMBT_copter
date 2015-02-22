#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define AILERON 3  // RX in pin
#define ELEVATOR 4 // RX in pin
#define RUDDER 7   // RX in pin
#define THROTTLE 6 // RX in pin

#define M1 14 // ESC motor pin 
#define M2 15 // ESC motor pin 
#define M3 16 // ESC motor pin 
#define M4 17 // ESC motor pin 

#define INVERT_CHANNEL 3000 // apply in 

MPU6050 mpu;
Servo motor[4];

// global vars
int16_t ax, ay, az; // raw values of the accelerometer output
int16_t gx, gy, gz; // raw values of the gyroscope output

unsigned long start_time = 0; // used to measure loop cycle time in ms

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
  
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
 
  // initialize serial comm
  Serial.begin(38400);
  
  // initialize mpu6050
  mpu.initialize();
  
  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

}

void loop() {  
  // start timing loop
  start_time = millis();
  
  // read in the 4 supported channels 
  int duration[4];
  duration[0] = pulseIn(AILERON, HIGH);
  duration[1] = INVERT_CHANNEL - pulseIn(ELEVATOR, HIGH);
  duration[2] = pulseIn(RUDDER, HIGH);
  duration[3] = INVERT_CHANNEL - pulseIn(THROTTLE, HIGH);
  
  // diplay miliseconds for each channel
  #if 0
    Serial.print("A/E/R/T = ");
    Serial.print(duration[0]);
    Serial.print("\t");
    Serial.print(duration[1]);
    Serial.print("\t");
    Serial.print(duration[2]);
    Serial.print("\t");
    Serial.println(duration[3]);
  #endif
  
  // relay the throttle to the motors
  motor[0].writeMicroseconds(duration[3]);
  motor[1].writeMicroseconds(duration[3]);
  motor[2].writeMicroseconds(duration[3]);
  motor[3].writeMicroseconds(duration[3]);
  
  // read raw acceletometer and gyro measurements from mpu6050
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // display raw mpu values
  #if 0
    Serial.print("a/g:\t");
    Serial.print(ax); Serial.print("\t");
    Serial.print(ay); Serial.print("\t");
    Serial.print(az); Serial.print("\t");
    Serial.print(gx); Serial.print("\t");
    Serial.print(gy); Serial.print("\t");
    Serial.println(gz);
  #endif
  
  // display time per loop cycle 
  #if 0
    Serial.print("loop cycle time = ");
    Serial.println(millis() - start_time);
  #endif
  
}

