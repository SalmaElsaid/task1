#include <ros.h>
#include <std_msgs/Int64.h>
#define input_A PB_0
#define input_B PB_1
long counter =0;
ros::NodeHandle  nh;
std_msgs::Int64 counter_msg;
ros::Publisher pub_count("counter",&counter_msg);

void setup() {

pinMode(input_A,INPUT_PULLUP);
pinMode(input_B,INPUT_PULLUP);
attachInterrupt(digitalPinToInterrupt(input_A),func_A,CHANGE);
attachInterrupt(digitalPinToInterrupt(input_B),func_B,CHANGE);


     nh.initNode();
     nh.advertise(pub_count);

}

void loop() {

 counter_msg.data=counter;
 pub_count.publish(&counter_msg);
 
  nh.spinOnce();
}
void func_A()
{
  if (digitalRead(input_A) != digitalRead(input_B))
  counter ++;
  else 
  counter --;
  
}

void func_B()
{
  if (digitalRead(input_A) <= digitalRead(input_B))
  counter ++;
  else 
  counter --;
}
