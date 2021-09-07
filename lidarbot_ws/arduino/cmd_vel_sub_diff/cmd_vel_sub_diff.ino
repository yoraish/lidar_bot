
#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>

// Pin variables for motors.
const int right_pwm_pin = 5;
const int right_dir_pin = A0;
const int left_pwm_pin = 6;
const int left_dir_pin = A1;
const bool left_fwd = false;
const bool right_fwd = true;

// Default_speed.
const int default_vel = 80;
int state_vel = default_vel;
const int max_vel = 255;

// Robot dimensions. In cm.
const float wheel_dist = 25.0;
ros::NodeHandle  nh;


// ROS stuff.

std_msgs::Int16 int_msg_right;
ros::Publisher motor_right_pub("/lidarbot/motor_right", &int_msg_right);

std_msgs::Int16 int_msg_left;
// ros::Publisher motor_left_pub("/lidarbot/motor_left", &int_msg_left);

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
  // Flipped r and l. Added steering scaler.
  float right_cmd = (-z_rotation*1.8)/2.0 + x;
  float left_cmd = 2.0*x - right_cmd;
  bool right_dir = (right_cmd>0)? right_fwd : !right_fwd;
  bool left_dir = (left_cmd>0)? left_fwd : !left_fwd;

  digitalWrite(right_dir_pin, right_dir);
  digitalWrite(left_dir_pin, left_dir);
  
  int right_write = int( default_vel * right_cmd);
  int left_write = int( default_vel * left_cmd );
 
  if (x == 0 && z_rotation == 0){
      MoveStop();
  }
  
  // Advertise the arduino command.
  int abs_left_write =  abs(left_write);
  int abs_right_write = abs(right_write);

  int_msg_right.data = right_write;
  int_msg_left.data = left_write;
  // motor_right_pub.publish(&int_msg_right);
  // motor_left_pub.publish(&int_msg_left);

  analogWrite(right_pwm_pin, abs_right_write);
  analogWrite(left_pwm_pin,  abs_left_write);
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
  //nh.advertise(motor_right_pub);
  //nh.advertise(motor_left_pub );


}

void loop() {
  nh.spinOnce();
  delay(1);
}
