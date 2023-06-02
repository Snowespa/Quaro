/* 
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
#include <std_msgs/Int16.h>
#include "lx_servo_control.hpp"
#include <SoftwareSerial.h>

const byte rxPin = 2;
const byte txPin = 3;
// Set up a new SoftwareSerial object
SoftwareSerial swSerial(rxPin, txPin);

ros::NodeHandle nh;

std_msgs::Int16 pos_msg;
ros::Publisher position("position", &pos_msg);

void setPosCb(const std_msgs::Int16& move_msg){
  lx_servo_serial_set_move_time(swSerial, ID00, move_msg.data, 0);
}

ros::Subscriber<std_msgs::Int16> sub("move", &setPosCb);

void setup()
{
    nh.initNode();
    nh.advertise(position);
    nh.subscribe(sub);
    nh.getHardware()->setBaud(BAUD);
    // Define pin modes for TX and RX
    pinMode(rxPin, INPUT);
    pinMode(txPin, OUTPUT);
    // Set the baud rate for the SoftwareSerial object
    swSerial.begin(BAUD);
    
    delay(10);

}

void loop()
{
  pos_msg.data = lx_servo_serial_get_pos(swSerial, ID00);
  position.publish(&pos_msg);
  nh.spinOnce();
  delay(10);
}

