#include <Wire.h>
#include <ros.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

#include <PWM.h> //PWM library for controlling the steppers

//Motor pins
//use pin 11 on the Mega instead, otherwise there is a frequency cap at 31 Hz
const int step_pin1 = 11;     // the pin that define the steps
const int dir_pin1 = 22;      // the dir pin
int32_t frequency1 = 0; //frequency (in Hz)

ros::NodeHandle nh;
std_msgs::Bool motor1_dir;
std_msgs::Float32 motor1_cmd;

//Motor_cmd Subscriber
void motor1_cmd_callback(const std_msgs::Float32& motor1_cmd){
  frequency1 = (int)motor1_cmd.data;  
}

//Motor_dir Subscriber
void motor1_dir_callback(const std_msgs::Bool& motor1_dir){
  if(motor1_dir.data == true){
    digitalWrite(dir_pin1, HIGH);
  }
  else{
    digitalWrite(dir_pin1, LOW);
  }
}

 ros::Subscriber<std_msgs::Float32> sub1("motor1_cmd", &motor1_cmd_callback );
 ros::Subscriber<std_msgs::Bool> sub2("motor1_dir", &motor1_cmd_callback );

void setup(void){ /* this is the same as void setup() but for older C and C++ */
  //initialize all timers except for 0, to save time keeping functions
  InitTimersSafe(); 
  
 nh.initNode();
 nh.subscribe(sub1);
 nh.subscribe(sub2);

  // =======================================================================================
  /* ================ PWM Function  ======================================================= */
  
  //PWM to the motors
  pinMode(step_pin1, OUTPUT);
  pinMode(dir_pin1, OUTPUT);
  SetPinFrequency(step_pin1, frequency1); //change for the callback function of motor1_cmd

  while(true){
    //setting the duty to 50% with the highest possible resolution that 
    //can be applied to the timer (up to 16 bit). 1/2 of 65536 is 32768.
    pwmWriteHR(step_pin1, 32768);
  }

}

void loop(void){ /* this is the same as void loop() but for older C and C++ */
  nh.spinOnce();
  delay(10);
}
