/*
* Ultrasonic Sensor HC-SR04 and Arduino Tutorial
*
* Crated by Dejan Nedelkovski,
* www.HowToMechatronics.com
*
*/
#include <ros.h>
#include <std_msgs/Float64.h>
#include <TimerOne.h>

ros::NodeHandle nh;
std_msgs::Float64 Distance;
ros::Publisher chatter("chatter",&Distance);

// defines pins numbers
const int trigPin = 9;
const int echoPin = 10;

// defines variables
long duration;
float distance;



void setup() 
{
  
  nh.initNode();
  nh.advertise(chatter);
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  Serial.begin(57600); // Starts the serial communication
  Timer1.initialize(50000); // set a timer of length 100000 microseconds (or 0.1 sec - or 10Hz => the led will blink 5 times, 5 cycles of on-and-off, per second)
  Timer1.attachInterrupt(callback); // attach the service routine here
}
void loop()
{
  // Clears the trigPin
 digitalWrite(trigPin, LOW); 
 delayMicroseconds(2);
 // Sets the trigPin on HIGH state for 10 micro seconds
 digitalWrite(trigPin, HIGH);
 delayMicroseconds(10);
 digitalWrite(trigPin, LOW);
 // Reads the echoPin, returns the sound wave travel time in microseconds
 duration = pulseIn(echoPin, HIGH);
 // Calculating the distance
 distance= duration*0.034/2;
 //publishing data
 Distance.data=distance;
}

void callback() 
{

 chatter.publish(&Distance);
 nh.spinOnce();
}
