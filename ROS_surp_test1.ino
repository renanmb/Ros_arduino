#include <Wire.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303.h>
#include <Adafruit_LSM303_U.h>
#include <Encoder.h>

#include <PWM.h> //PWM library for controlling the steppers
//#include <inttypes.h> //not sure about this for the int32_t

/* Assign a unique ID to the sensors */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
//******** add later gyroscope and magnet sensor for more complex positioning

//  Both pins must have interrupt capability
Encoder myEnc(2,3);

ros::NodeHandle  nh;
std_msgs::Int32 encoder;
std_msgs::Float32 velLinear_x;
std_msgs::Float32 velAngular_z;
ros::Publisher velLinear_x_pub("velLinear_x" , &velLinear_x);
ros::Publisher velAngular_z_pub("velAngular_z" , &velAngular_z);
ros::Publisher encoder_pub("encoder" , &encoder);


//Motor pins
//use pin 11 on the Mega instead, otherwise there is a frequency cap at 31 Hz
const int step_pin1 = 11;     // the pin that define the steps
const int dir_pin1 = 22;      // the dir pin
const int step_pin2 = 12;     // the pin that define the steps
const int dir_pin2 = 24;
int32_t frequency1 = 0; //frequency (in Hz)
int32_t frequency2 = 0; //frequency (in Hz)
int32_t duty1 = 0;
int32_t duty2 = 0;
long oldPosition = -999;

void driveCallback ( const geometry_msgs::Twist&  twistMsg ){
  //need to pubish straigth the motor commands and do the dir calculations on the Arduino
  frequency1 = abs( twistMsg.linear.x ) ;//work as motor1_cmd_callback ---- twistMsg.linear.x must already be converted to int32_t
  frequency2 = abs( twistMsg.linear.y ) ;//work as motor2_cmd_callback ---- twistMsg.linear.y must already be converted to int32_t
  SetPinFrequency(step_pin1, frequency1); //change for the callback function of motor1_cmd
  SetPinFrequency(step_pin2, frequency2); //change for the callback function of motor2_cmd

  if(frequency1 > 0){
    duty1 = 32768; 
    if(twistMsg.linear.x >= 0){
    digitalWrite(dir_pin1, HIGH);
    }
    else{
    digitalWrite(dir_pin1, LOW);
    }
  }
  else{
    duty1 = 0;
  }

  if(frequency2 > 0){
    duty2 = 32768;
    if(twistMsg.linear.y >= 0){
    digitalWrite(dir_pin2, LOW);
    }
    else{
    digitalWrite(dir_pin2, HIGH);
    }
  }
  else{
    duty2 = 0;
  }
}

void initSensors(){
   if(!accel.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }
}

ros::Subscriber<geometry_msgs::Twist> driveSubscriber("comm_drive_robot", &driveCallback) ;

void setup(){
  //initialize all timers except for 0, to save time keeping functions
  InitTimersSafe(); 
  nh.initNode();

  // Subscribe to the steering and throttle messages
  nh.subscribe(driveSubscriber);
  nh.advertise(velLinear_x_pub);
  nh.advertise(velAngular_z_pub);
  nh.advertise(encoder_pub);

  // =======================================================================================
  /* ================ PWM Function  ======================================================= */
  //PWM + direction to the motors 1
  pinMode(step_pin1, OUTPUT);
  pinMode(dir_pin1, OUTPUT);
  
  //PWM + direction to the motors 2
  pinMode(step_pin2, OUTPUT);
  pinMode(dir_pin2, OUTPUT);
  
  //initialize the sensor and its bandwidth here
   initSensors();
}

void loop(){
   /*create here the variables to store de data received from the sensor.*/
  /* Objects */
  sensors_event_t accel_event;

  /* Read the accelerometer */
  accel.getEvent(&accel_event);

  /* How do I insert in the right units ? */
  velLinear_x.data = accel_event.acceleration.x ;
  velAngular_z.data = accel_event.acceleration.y ;

  /* Encoder reading - long in arduino is the same as int32
     This code gives a int32 number that can be copared to get speed
     myEnc.*/
  long newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    encoder.data = newPosition;
  }

  velLinear_x_pub.publish(&velLinear_x);
  velAngular_z_pub.publish(&velAngular_z);
  encoder_pub.publish(&encoder);
  
  nh.spinOnce();

  pwmWriteHR(step_pin1, duty1);
  pwmWriteHR(step_pin2, duty2);
  
}
