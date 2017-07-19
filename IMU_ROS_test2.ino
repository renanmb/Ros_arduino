//#define USE_USBCON
#include <Wire.h>

//#include <ArduinoHardware.h>
#include <ros.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303.h>
#include <Adafruit_LSM303_U.h>

#include <std_msgs/Float32.h>


/* Assign a unique ID to the sensors */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);

ros::NodeHandle nh;
std_msgs::Float32 velLinear_x;
std_msgs::Float32 velAngular_z;

ros::Publisher velLinear_x_pub("velLinear_x" , &velLinear_x);
ros::Publisher velAngular_z_pub("velAngular_z" , &velAngular_z);

void initSensors(){
   if(!accel.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }
}

void setup(void){ /* this is the same as void setup() but for older C and C++ */
  nh.initNode();
  nh.advertise(velLinear_x_pub);
  nh.advertise(velAngular_z_pub);

//  wire.begin();
//  delay(100);
   //initialize the sensor and its bandwidth here
   initSensors();
}

void loop(void){ /* this is the same as void loop() but for older C and C++ */
  
  /*create here the variables to store de data received from the sensor.*/
  /* Objects */
  sensors_event_t accel_event;

  /* Read the accelerometer */
  accel.getEvent(&accel_event);

  /* How do I insert in the right units ? */
  velLinear_x.data = accel_event.acceleration.x ;
  velAngular_z.data = accel_event.acceleration.y ;

  velLinear_x_pub.publish(&velLinear_x);
  velAngular_z_pub.publish(&velAngular_z);

  nh.spinOnce();
  delay(10);
}
  

