#include <Wire.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>

#include <PWM.h> //PWM library for controlling the steppers
//#include <inttypes.h> //not sure about this for the int32_t


ros::NodeHandle  nh;
std_msgs::Int32 str_msg;
ros::Publisher chatter("debug", &str_msg);

//Motor pins
//use pin 11 on the Mega instead, otherwise there is a frequency cap at 31 Hz
const int step_pin1 = 11;     // the pin that define the steps
const int dir_pin1 = 22;      // the dir pin
const int step_pin2 = 12;     // the pin that define the steps
const int dir_pin2 = 24;
int32_t frequency1 = 1000; //frequency (in Hz)
int32_t frequency2 = 0; //frequency (in Hz)
bool dir1 = true;
bool dir2 = true;

void driveCallback ( const geometry_msgs::Twist&  twistMsg ){
  //need to pubish straigth the motor commands and do the dir calculations on the Arduino
  frequency1 = abs( (int)twistMsg.linear.x ) ;//work as motor1_cmd_callback ---- twistMsg.linear.x must already be converted to int32_t
  frequency2 = abs( (int)twistMsg.linear.y ) ;//work as motor2_cmd_callback ---- twistMsg.linear.y must already be converted to int32_t
 // SetPinFrequency(step_pin1, frequency1); //change for the callback function of motor1_cmd
 // SetPinFrequency(step_pin2, frequency2); //change for the callback function of motor2_cmd

  str_msg.data = frequency1;
  chatter.publish(&str_msg);
  
  if(twistMsg.linear.x >= 0){
    dir1 = true;
  //digitalWrite(dir_pin1, HIGH);
  }
  else{
    dir1 = false;
  //digitalWrite(dir_pin1, LOW);
  }
  
  if(twistMsg.linear.y >= 0){
    dir2 = true;
  //digitalWrite(dir_pin2, HIGH);
  }
  else{
    dir2 = false;
  //digitalWrite(dir_pin2, LOW);
  }

}

ros::Subscriber<geometry_msgs::Twist> driveSubscriber("comm_drive_robot", &driveCallback) ;

void setup(){
  //initialize all timers except for 0, to save time keeping functions
  InitTimersSafe(); 
  nh.initNode();

  // Subscribe to the steering and throttle messages
  nh.subscribe(driveSubscriber);
  nh.advertise(chatter);

  // =======================================================================================
  /* ================ PWM Function  ======================================================= */
  //PWM + direction to the motors 1
  pinMode(step_pin1, OUTPUT);
  pinMode(dir_pin1, OUTPUT);
  
  //PWM + direction to the motors 2
  pinMode(step_pin2, OUTPUT);
  pinMode(dir_pin2, OUTPUT);
  //SetPinFrequency(step_pin1, frequency1); //change for the callback function of motor1_cmd
  //SetPinFrequency(step_pin2, frequency2); //change for the callback function of motor2_cmd

  if(dir1 == true){
    digitalWrite(dir_pin1, HIGH);
    SetPinFrequency(step_pin1, frequency1); //change for the callback function of motor1_cmd
  }
  else{
    digitalWrite(dir_pin1, LOW);
    SetPinFrequency(step_pin1, frequency1); //change for the callback function of motor1_cmd
  }
  if(dir2 == false){
    digitalWrite(dir_pin2, HIGH);
    SetPinFrequency(step_pin2, frequency2); //change for the callback function of motor2_cmd
  }
  else{
    digitalWrite(dir_pin1, LOW);
    SetPinFrequency(step_pin2, frequency2); //change for the callback function of motor2_cmd
  }
  
}

void loop(){
  nh.spinOnce();
  delay(1);
}
