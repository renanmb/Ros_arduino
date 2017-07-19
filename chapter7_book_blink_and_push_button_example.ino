#include <ros.h>
#include <std_msgs/Bool.h>

//Nodehandle
ros::NodeHandle nh;

//Boolean message for Push button
std_msgs::Bool pushed_msg;

//defining publisher in a topic called pushed
ros::Publisher pub_button("pushed", &pushed_msg);

//LED and Push button pin definitions
const int button_pin = 7 ;
const int led_pin = 13 ;

//variables to handle debouncing
//https: //www.arduino.cc/en/Tutorial/Debounce

bool last_reading;
long last_debounce_time = 0;
long debounce_delay = 50;
bool published = true;

void setup(){
  nh.initNode();
  nh.advertise(pub_button);

  //initialize an LED output pin
  //and an input pin for our push button
  pinMode(led_pin, OUTPUT);
  pinMode(button_pin, INPUT);

  //Enable pull up resistor on the button
  digitalWrite(button_pin, HIGH);

  //The button is a normally button
  last_reading = ! digitalRead(button_pin);
  
}

void loop(){
  bool reading = ! digitalRead(button_pin);

  if (last_reading!= reading){
    last_debounce_time = millis();
    published = false;
  }

  //if the button value has not changed for the debounce delay, we know its stable
  if( !published && (millis() - last_debounce_time) > debounce_delay){
    digitalWrite(led_pin, reading);
    pushed_msg.data = reading;
    pub_button.publish(&pushed_msg);
    published = true;
  }

  last_reading = reading;

  nh.spinOnce();
}

