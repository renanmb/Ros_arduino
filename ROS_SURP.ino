#include <Wire.h>
#include <ros.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303.h>
#include <Adafruit_LSM303_U.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

#include <Encoder.h> //encoder library - http://www.pjrc.com/teensy/td_libs_Encoder.html

#include <PWM.h> //PWM library for controlling the steppers

/* Assign a unique ID to the sensors */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
//******** add later gyroscope and magnet sensor for more complex positioning


//  Both pins must have interrupt capability
Encoder myEnc(2,3);

//Motor pins
//use pin 11 on the Mega instead, otherwise there is a frequency cap at 31 Hz
const int step_pin1 = 9;     // the pin that define the steps
const int dir_pin1 = 7;      // the dir pin
const int step_pin2 = 10;     // the pin that define the steps
const int dir_pin2 = 8;
int32_t frequency = 5000; //frequency (in Hz)

ros::NodeHandle nh;
std_msgs::Float32 velLinear_x;
std_msgs::Float32 velAngular_z;
std_msgs::Int32 encoder;

ros::Publisher velLinear_x_pub("velLinear_x" , &velLinear_x);
ros::Publisher velAngular_z_pub("velAngular_z" , &velAngular_z);
ros::Publisher encoder_pub("encoder" , &encoder);

void initSensors(){
   if(!accel.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }
}

void messageCb( const std_msgs::Int32& frequency){
   //Write the PWM frequency for the motors
  digitalWrite(dir_pin1, HIGH);
  digitalWrite(dir_pin2, HIGH);
  //this loop can be a issue if the whole program stays at it
  while(true){
    //setting the duty to 50% with the highest possible resolution that 
    //can be applied to the timer (up to 16 bit). 1/2 of 65536 is 32768.
    pwmWriteHR(step_pin1, 32768);
    pwmWriteHR(step_pin2, 32768);
  }
}

 ros::Subscriber<std_msgs::Int32> sub("???????", &messageCb );

void setup(void){ /* this is the same as void setup() but for older C and C++ */
  //initialize all timers except for 0, to save time keeping functions
  InitTimersSafe(); 
  
  nh.initNode();
  nh.advertise(velLinear_x_pub);
  nh.advertise(velAngular_z_pub);
  nh.advertise(encoder_pub);
  
  //PWM to the motors
  pinMode(step_pin1, OUTPUT);
  pinMode(dir_pin1, OUTPUT);
  SetPinFrequency(step_pin1, frequency); //setting the frequency to 10 Hz
  pinMode(step_pin2, OUTPUT);
  pinMode(dir_pin2, OUTPUT);
  SetPinFrequency(step_pin2, frequency);

   //initialize the sensor and its bandwidth here
   initSensors();
}

long oldPosition  = -999;

void loop(void){ /* this is the same as void loop() but for older C and C++ */
  
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
  delay(10);
}
