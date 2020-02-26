
#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include <ros.h>
#include <geometry_msgs/Twist.h>

// Pin variables for motors.
const int right_pwm_pin = 5;
const int right_dir_pin = A0;
const int left_pwm_pin = 6;
const int left_dir_pin = A1;
const bool left_fwd = true;
const bool right_fwd = false;

// Default_speed.
const int default_vel = 100;
int state_vel = default_vel;
const int max_vel = 255;

ros::NodeHandle  nh;


void MoveStop() {
  digitalWrite(right_dir_pin, right_fwd);
  digitalWrite(left_dir_pin, left_fwd);
  analogWrite(right_pwm_pin, 0);
  analogWrite(left_pwm_pin, 0);
}



void cmd_vel_cb(const geometry_msgs::Twist & msg) {
  // Read the message. Act accordingly.
  // We only care about the linear x, and the rotational z.
  const float x = msg.linear.x;
  const float z_rotation = msg.angular.z;
  int right_cmd = x*default_vel * min(1, max(z_rotation*1.5 + 1, -1));
  int left_cmd =  x*default_vel * min(1, max(-z_rotation*1.5 + 1 , -1));
  bool right_dir = (right_cmd>0)? right_fwd : !right_fwd;
  bool left_dir = (left_cmd>0)? left_fwd : !left_fwd;

  digitalWrite(right_dir_pin, right_dir);
  digitalWrite(left_dir_pin, left_dir);

  analogWrite(right_pwm_pin, abs(right_cmd));
  analogWrite(left_pwm_pin, abs(left_cmd));

  if (x == 0){
      MoveStop();
  }
}


ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmd_vel_cb);

void setup() {

  pinMode(right_pwm_pin, OUTPUT);    // sets the digital pin 13 as output
  pinMode(right_dir_pin, OUTPUT);
  pinMode(left_pwm_pin, OUTPUT);
  pinMode(left_dir_pin, OUTPUT);
  // Set initial values for directions. Set both to forward.
  digitalWrite(right_dir_pin, right_fwd);
  digitalWrite(left_dir_pin, left_fwd);
  pinMode(13, OUTPUT);
  // Send forward command.
  analogWrite(right_pwm_pin, default_vel);
  analogWrite(left_pwm_pin, default_vel);
  delay(500);
  MoveStop();

  nh.initNode();
  nh.subscribe(sub);



}

void loop() {
  nh.spinOnce();
  delay(1);
}
