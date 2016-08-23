#include <ros.h>
#include <AccelStepper.h>
#include <cb1_msgs/MoveTurntable.h>
#include <std_msgs/Int32.h>

AccelStepper stepper(2,12,13);

const int pwmA = 3;
const int pwmB = 11;
const int brakeA = 8;
const int brakeB = 9;

int goal_pos = 0;
long interval = 50;
long previous_millis = 0;
ros::NodeHandle  nh;

void callback(const cb1_msgs::MoveTurntable::Request & req, cb1_msgs::MoveTurntable::Response & res){
  goal_pos += req.steps;
  stepper.moveTo(goal_pos);
  while (stepper.distanceToGo() > 0){
    nh.spinOnce();
    stepper.run();
  }
  res.result = true;

}

ros::ServiceServer<cb1_msgs::MoveTurntable::Request, cb1_msgs::MoveTurntable::Response> server("move_turntable",&callback);

void setup()
{  
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertiseService(server);
  pinMode(pwmA, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(brakeA, OUTPUT);
  pinMode(brakeB, OUTPUT);
  
  digitalWrite(pwmA, HIGH);
  digitalWrite(pwmB, HIGH);
  digitalWrite(brakeA, LOW);
  digitalWrite(brakeB, LOW);
  
  stepper.setMaxSpeed(100);
  stepper.setSpeed(100);
  stepper.setAcceleration(200);
    
}

void loop(){  
  nh.spinOnce();
  stepper.run();
  delay(10);
}
