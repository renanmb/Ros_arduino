#include <Wire.h>
#include <ros.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303.h>
#include <Adafruit_LSM303_U.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

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
int32_t frequency1 = 0; //frequency (in Hz)
int32_t frequency2 = 0; //frequency (in Hz)

ros::NodeHandle nh;
std_msgs::Float32 velLinear_x;
std_msgs::Float32 velAngular_z;
std_msgs::Int32 encoder;
std_msgs::Bool motor1_dir;
std_msgs::Bool motor2_dir;
std_msgs::Float32 motor1_cmd;
std_msgs::Float32 motor2_cmd

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

void motor1_cmd_callback(const std_msgs::Float32& motor1_cmd){
  frequency1 = (int)motor1_cmd.data;  
}

void motor2_cmd_callback(const std_msgs::Float32& motor2_cmd){
  frequency2 = (int)motor2_cmd.data;
}

void motor1_dir_callback(const std_msgs::Bool& motor1_dir){
  if(motor1_dir.data == True){
    digitalWrite(dir_pin1, HIGH);
  }
  else{
    digitalWrite(dir_pin1, LOW);
  }
}

void motor2_dir_callback(const std_msgs::Bool& motor2_dir){
  if(motor2_dir.data == True){
    digitalWrite(dir_pin2, HIGH);
  }
  else{
    digitalWrite(dir_pin2, LOW);
  }
}

 ros::Subscriber<std_msgs::Float32> sub("motor1_cmd", &motor1_cmd_callback );
 ros::Subscriber<std_msgs::Float32> sub("motor2_cmd", &motor2_cmd_callback );
 ros::Subscriber<std_msgs::Bool> sub("motor1_dir", &motor1_cmd_callback );
 ros::Subscriber<std_msgs::Bool> sub("motor2_dir", &motor2_cmd_callback );

void setup(void){ /* this is the same as void setup() but for older C and C++ */
  //initialize all timers except for 0, to save time keeping functions
  InitTimersSafe(); 
  
  nh.initNode();
  nh.advertise(velLinear_x_pub);
  nh.advertise(velAngular_z_pub);
  nh.advertise(encoder_pub);
  nh.subscribe(sub);

  // =======================================================================================
  /* ================ PWM Function  ======================================================= */
  
  //PWM to the motors
  pinMode(step_pin1, OUTPUT);
  pinMode(dir_pin1, OUTPUT);
  SetPinFrequency(step_pin1, frequency1); //change for the callback function of motor1_cmd
  pinMode(step_pin2, OUTPUT);
  pinMode(dir_pin2, OUTPUT);
  SetPinFrequency(step_pin2, frequency2); //change for the callback function of motor2_cmd

  while(true){
    //setting the duty to 50% with the highest possible resolution that 
    //can be applied to the timer (up to 16 bit). 1/2 of 65536 is 32768.
    pwmWriteHR(step_pin1, 32768);
    pwmWriteHR(step_pin2, 32768);
  }

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
