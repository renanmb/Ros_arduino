#include <PWM.h>

//use pin 11 on the Mega instead, otherwise there is a frequency cap at 31 Hz
const int step_pin1 = 9;     // the pin that define the steps
const int dir_pin1 = 7;      // the dir pin
const int step_pin2 = 10;     // the pin that define the steps
const int dir_pin2 = 8;
int32_t frequency = 5000; //frequency (in Hz)

void setup()
{
  //initialize all timers except for 0, to save time keeping functions
  InitTimersSafe(); 
  
  pinMode(step_pin1, OUTPUT);
  pinMode(dir_pin1, OUTPUT);
  SetPinFrequency(step_pin1, frequency); //setting the frequency to 10 Hz
  pinMode(step_pin2, OUTPUT);
  pinMode(dir_pin2, OUTPUT);
  SetPinFrequency(step_pin2, frequency);

}

void loop(){
  digitalWrite(dir_pin1, HIGH);
  digitalWrite(dir_pin2, HIGH);
  while(true){
    //setting the duty to 50% with the highest possible resolution that 
    //can be applied to the timer (up to 16 bit). 1/2 of 65536 is 32768.
    pwmWriteHR(step_pin1, 32768);
    pwmWriteHR(step_pin2, 32768);
  }
  

     
}

