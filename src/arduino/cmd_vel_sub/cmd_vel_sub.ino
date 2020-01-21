
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
enum State {FWD, BWD, RIGHT, LEFT, STOP};
State state;
const int max_vel = 255;

ros::NodeHandle  nh;


void MoveLeft(const size_t speed) {
  digitalWrite(right_dir_pin, right_fwd);
  digitalWrite(left_dir_pin, !left_fwd);
  analogWrite(right_pwm_pin, speed);
  analogWrite(left_pwm_pin, speed);
}

void MoveRight(const size_t speed) {
  digitalWrite(right_dir_pin, !right_fwd);
  digitalWrite(left_dir_pin, left_fwd);
  analogWrite(right_pwm_pin, speed);
  analogWrite(left_pwm_pin, speed);
}

void MoveFwd(const size_t speed) {
  digitalWrite(right_dir_pin, right_fwd);
  digitalWrite(left_dir_pin, left_fwd);
  analogWrite(right_pwm_pin, speed);
  analogWrite(left_pwm_pin, speed);
}

void MoveBwd(const size_t speed) {
  digitalWrite(right_dir_pin, !right_fwd);
  digitalWrite(left_dir_pin, !left_fwd);
  analogWrite(right_pwm_pin, speed);
  analogWrite(left_pwm_pin, speed);
}

void MoveStop() {
  digitalWrite(right_dir_pin, right_fwd);
  digitalWrite(left_dir_pin, left_fwd);
  analogWrite(right_pwm_pin, 0);
  analogWrite(left_pwm_pin, 0);
}


void MoveDifferential(float x, float z_rot) {

  // x determines overall intensity. The max intensity is 255 to each motor, summing to 510 units of awesome.
  // z_rot determines how much we turn. +1 is completely left [-x*255, +x*255]. -1 is completely right [x*255, -x*255].
  // So we use the equations: (if z_rot !=0)
  // If z_rot > 0:
  //      Right = x*255
  //      Left = 255*x - 510*z_rot*x
  // If z_rot < 0:
  //      Right = 255*x - 510*|z_rot|*x
  //      Left = 255*x

  float right_vel = 0;
  float left_vel = 0;
  if (z_rot == 0.0) {
    right_vel = max_vel * x;
    left_vel = max_vel * x;
  }
  else if (z_rot > 0.0) {
    right_vel = max_vel * x;
    left_vel = max_vel * x - 2.0 * max_vel * z_rot * x;
  }
  else if (z_rot > 0.0) {
    left_vel = max_vel * x;
    right_vel = max_vel * x - 2 * max_vel * abs(z_rot) * x;
  }
  else {
    right_vel = 0.0;
    left_vel = 0.0;
  }

  if (x == 0 && z_rot != 0) {
    right_vel = (z_rot > 0) ? z_rot * max_vel : -z_rot * max_vel;
    left_vel  = (z_rot > 0) ? -z_rot * max_vel : z_rot * max_vel;
  }


  digitalWrite(right_dir_pin, (right_vel >= 0) ? right_fwd : !right_fwd);
  digitalWrite(left_dir_pin, (left_vel >= 0) ? left_fwd : !left_fwd);
  analogWrite(right_pwm_pin, abs((int) right_vel));
  analogWrite(left_pwm_pin, abs((int) right_vel));
}

void cmd_vel_cb(const geometry_msgs::Twist & msg) {
  // Read the message. Act accordingly.
  // We only care about the linear x, and the rotational z.
  const float x = msg.linear.x;
  const float z_rotation = msg.angular.z;
  //  MoveDifferential(x,z_rotation);
  if (x > 0) {
    if (state == FWD){
      state_vel +=10;
      state_vel = min(state_vel, 255);
    }
    MoveFwd(state_vel);
    state = FWD;
  }
  else if (x < 0) {
    if (state == BWD){
      state_vel +=10;
      state_vel = min(state_vel, 255);

    }
    MoveBwd(state_vel);
    state = BWD;
  }
  else if (z_rotation < 0) {
    if (state == RIGHT){
      state_vel +=10;
      state_vel = min(state_vel, 255);

    }
    MoveRight (state_vel);
    state = RIGHT;
  }  
  else if (z_rotation > 0) {
    if (state == LEFT){
      state_vel += 10;
      state_vel = min(state_vel, 255);

    }
    MoveLeft (state_vel);
    state = LEFT;
  }
  else {
    MoveStop();
    state_vel = default_vel;
    state = STOP;
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
  MoveFwd(200);
  delay(500);
  MoveRight(180);
  delay(100);
  MoveStop();

  nh.initNode();
  nh.subscribe(sub);



}

void loop() {
  nh.spinOnce();
  delay(1);
}
