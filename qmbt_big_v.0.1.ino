#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <math.h>

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

uint32_t timer; // used to keep track of time
unsigned long start_time = 0; // used to measure loop cycle time in ms

double raw_x_degree = 0; // raw x angle in degrees
double raw_y_degree = 0; // raw y angle in degrees
double raw_z_degree = 0; // raw z angle in degrees

double comp_x_degree = 0; // complimentary x angle in degrees
double comp_y_degree = 0; // complimentary y angle in degrees
double comp_z_degree = 0; // complimentary z angle in degrees

double acc_x_degree = 0; // angle estimated using the accelerometer 
double acc_y_degree = 0; // angle estimated using the accelerometer

int rx_duration_last[4] = {1500,1500,1500,1000}; // used to average channels and for comparesment - safety mechanism
int rx_duration[4]; // store the ms each channel output

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
  
  // should be ran only once and only if needed.
  //    i.e. if there is too much noise on the acc or gryo
  // set gyroscope and accelerometer ranges
  #if 0
  accelgyro.setFullScaleGyroRange(0x01);
  accelgyro.setFullScaleAccelRange(0x01); 
  accelgyro.setDLPFMode(0x06);
  
  Serial.print("gyro rate: ");
  Serial.println(accelgyro.getFullScaleGyroRange());
  Serial.print("accel rate: ");
  Serial.println(accelgyro.getFullScaleAccelRange());
  #endif
  
  // initialize the timer 
  timer = micros(); 
}

void loop() {  
  // start timing loop
  start_time = millis();
    
  // get the user input
  getUserInput();
  
  // populate new filtered values
  callIMU();
  
  // relay the throttle to the motors
  motor[0].writeMicroseconds(rx_duration[3]);
  motor[1].writeMicroseconds(rx_duration[3]);
  motor[2].writeMicroseconds(rx_duration[3]);
  motor[3].writeMicroseconds(rx_duration[3]);
       
  // display time per loop cycle 
  #if 0
    Serial.print("loop cycle time = ");
    Serial.println(millis() - start_time);
  #endif
  
}

void callIMU()
{
  // read raw acceletometer and gyro measurements from mpu6050
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // check how much time has gone by
  double dt = (double)(micros() - timer) / 1000000; 
  
  // update the timer
  timer = micros(); 
  
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
  
  // calculate the angles 
  
  // convert to degrees per second
  double gyro_x_rate = gx / 131.0; 
  double gyro_y_rate = gy / 131.0; 
  double gyro_z_rate = gz / 131.0;
  
  // display degrees per second of the xyz axies
  #if 0
    Serial.print("gx/gy/gz in deg/sec = ");
    Serial.print(gyro_x_rate); Serial.print("\t");
    Serial.print(gyro_y_rate); Serial.print("\t");
    Serial.println(gyro_z_rate);
  #endif
  
  // raw gyro values in degrees
  raw_x_degree += gyro_x_rate * dt; // Calculate gyro angle without any filter
  raw_y_degree += gyro_y_rate * dt;
  raw_z_degree += gyro_z_rate * dt;
  
  // display raw degrees of the xyz axies (non-filtered)
  #if 0
    Serial.print("gx/gy/gz in deg = ");
    Serial.print(raw_x_degree); Serial.print("\t");
    Serial.print(raw_y_degree); Serial.print("\t");
    Serial.println(raw_z_degree);
  #endif
  
  // convert to angles using the accelerometer  
  acc_x_degree = atan((double) ay / sqrt((double) ax * (double) ax + (double) az * (double) az)) * RAD_TO_DEG;
  acc_y_degree = atan2((double) -ax, (double) az) * RAD_TO_DEG;
  
  // display angles estimated using the accelerameter
  #if 0
    Serial.print("ax/ay in deg = ");
    Serial.print(acc_x_degree); Serial.print("\t");
    Serial.println(acc_y_degree);
  #endif
  
  // apply complimentary filtering - kalman is too hefty for this this mcu when using pulseIn :( 
  comp_x_degree = 0.93 * (comp_x_degree + gyro_x_rate * dt) + 0.07 * acc_x_degree; // Calculate the angle using a Complimentary filter
  comp_y_degree = 0.93 * (comp_y_degree + gyro_y_rate * dt) + 0.07 * acc_y_degree;
  comp_z_degree = raw_z_degree; // no magnometer thus we cannot filter the z axis. :(
  
  // display angles estimated using the complimentary filter
  #if 0
    Serial.print("compl angles x/y/z  = ");
    Serial.print(comp_x_degree); Serial.print("\t");
    Serial.print(comp_y_degree); Serial.print("\t");
    Serial.println(comp_z_degree);
  #endif
    
}

void getUserInput()
{
  // check whats going on all the rx channels  
  // read in the 4 supported channels 
  rx_duration[0] = pulseIn(AILERON, HIGH);
  rx_duration[1] = INVERT_CHANNEL - pulseIn(ELEVATOR, HIGH);
  rx_duration[2] = pulseIn(RUDDER, HIGH);
  rx_duration[3] = INVERT_CHANNEL - pulseIn(THROTTLE, HIGH);
  
  // save teh rx duration before modifying
  int rx_temp[4];
  rx_temp[0] = rx_duration[0];
  rx_temp[1] = rx_duration[1];
  rx_temp[2] = rx_duration[2];
  rx_temp[3] = rx_duration[3];
  
  // smoothing and safety algorithm - average of the last 2 - average of the last 3 would probably better (or even a filter)
  for(int i = 0; i < 4; i++)
  {
    if(rx_duration[i] - rx_duration_last[i] > 1000)
    {
      // this is not normal. disable the quadcopter
      // not implemented for now. 
    }
    else
    {
      // average the two values together
      rx_duration[i] = (rx_duration[i] + rx_duration_last[i])/2;     
      
      // save the current value to the old one
      rx_duration_last[i] = rx_temp[i];
    }
  }
   
  // diplay miliseconds for each channel
  #if 0
    Serial.print("A/E/R/T = ");
    Serial.print(rx_duration[0]);
    Serial.print("\t");
    Serial.print(rx_duration[1]);
    Serial.print("\t");
    Serial.print(rx_duration[2]);
    Serial.print("\t");
    Serial.println(rx_duration[3]);
  #endif
}

