#include <ros.h>
#include <std_msgs/Float64.h>

#define input_A PB12
#define input_B PB13

int counter = 0;
const float kp = 0, ki = 0, kd = 0;
double error = 0 ;
double last_error = 0 ;
long current_time = 0;
long previous_time = 0;
long elapsed_time = 0;
double integtal_Error, deriv_Error;
int interval = 1000;
float ang_velocity = 0;
float ang_velocity_deg = 0;
float rpm = 0;
const int REV = 620;
const float rpm_to_rad = 0.104719;
const float rad_to_deg = 57.29578;
double Setpoint;


ros::NodeHandle  nh;
void setpoint_msg( const std_msgs::Float64& msg){

}
ros::Subscriber <std_msgs::Float64>  setpoint("setpoint", &setpoint_msg);



void setup() {
  pinMode(input_A, INPUT_PULLUP);
  pinMode(input_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(input_A), func_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(input_B), func_B, CHANGE);


  (nh.getHardware()->setPort(&Serial1));
  (nh.getHardware()->setBaud(115200));


  nh.initNode();
  nh.subscribe(setpoint);
}

void loop() {
  double calc_velocity();
  current_time = millis();
  elapsed_time = current_time - previous_time;
  error = Setpoint - ang_velocity_deg;
  integtal_Error += error * elapsed_time;
  deriv_Error = (error - last_error) / elapsed_time;

  double Speed = kp * error + ki * integtal_Error + kd * deriv_Error ;         //PID output

  last_error = error;                                      //remember current error
  previous_time = current_time;



  nh.spinOnce();
  delay(10);
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
  if (digitalRead(input_A) == digitalRead(input_B))
    counter ++;
  else
    counter --;
}
double calc_velocity()
{
  current_time = millis();


  if (current_time - previous_time > interval) {

    previous_time = current_time;

    rpm = (counter * 60 / REV);
    ang_velocity = rpm * rpm_to_rad;   
    ang_velocity_deg = ang_velocity * rad_to_deg;
    counter =0;
    return ang_velocity_deg;
  }
  }
