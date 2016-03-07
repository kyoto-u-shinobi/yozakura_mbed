// (C) 2015 Kyoto University Mechatronics Laboratory
// Released under the GNU General Public License, version 3
#include "mbed.h"
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>

#define N_MOTOR 4

// Class representing a motor driver.
//
// Connect the PWM and DIR pins to the mbed. The motor driver's fault signals
// should go to the Raspberry Pi. Ground can be connected to either.
//
// Datasheet: https://www.pololu.com/product/755
//
// Examples:
//     Motor motor(p21, p11, p6);
//     motor.drive(0.5) // Runs motor forward at 50% speed.
//     motor.drive(-0.5) // Runs motor backwards at 50% speed.
class Motor {
 public:
  // Initialize the motor.
  //
  // Parameters:
  //   pin_pwm: The motor driver's PWM pin. In order to have PWM output, the
  //            pin should be between 21 and 26.
  //   pin_dir: The motor driver's DIR pin. If the driver is not connected in
  //            reverse, HI is forward, and LO is reverse.
  //   reversed: Whether the motor driver's DIR pin is connected in reverse.
  Motor(PinName pin_pwm, PinName pin_dir, PinName pin_enc, bool reversed)
      : pwm_(pin_pwm), dir_(pin_dir), enc_(pin_enc), reversed_(reversed) {
    pwm_ = 0.0;
    dir_ = 0;
    pwm_.period_us(40);  // Set PWM output frequency to 25 kHz.
     
    count_ = 0;
    velocity_ = 0.0;
    weight_ = 0.1;
    enc_.rise(this, &Motor::rise_);
    count2vel.attach(this, &Motor::calcVelocity_, 0.1);   
  }

  // Drive the motor at the given speed.
  //
  // Parameters:
  //   speed: A value between -1 and 1, representing the motor speed.
  void Drive(float speed) {
    if (reversed_) {
      dir_ = speed < 0 ? 1 : 0;
    } else {
      dir_ = speed < 0 ? 0 : 1;
    }
    pwm_ = fabs(speed);
  }
  
  // Callback function for encoder interrupt
  void rise_() {
    count_ += direction_();
  }
  
  // Guess the direction from command
  int direction_() {
    return dir_ == 0 ? -1 : 1;
  }
  
  // Get calculated velocity
  float getVelocity() {
    return velocity_;
  }
  
  // Filter the counter value  
  void calcVelocity_() {
    velocity_ = weight_*count_ + (1-weight_)*velocity_;
    count_ = 0;
  }

 private:
    PwmOut pwm_;      // The motor driver's PWM pin.
    DigitalOut dir_;  // The motor driver's DIR pin.
    bool reversed_;   // Whether DIR is connected in reverse.

    InterruptIn enc_;           // The encorder's pin.    
    Ticker count2vel;           // Timer
    int count_;                 
    float velocity_, weight_;   
};

// The four motors are in an array. The raspberry pi expects this order; do
// not change it without changing the code for the RPi as well.
Motor motors[N_MOTOR] = { Motor(p26, p27, p8, true),      // Right front wheel
                          Motor(p25, p28, p7, false),     // Left front wheel
                          Motor(p24, p29, p6, true),      // Right back wheel
                          Motor(p23, p30, p5, false) };   // Left back wheel

float safety_limit = 0.3;

ros::NodeHandle nh;

void commandCb(const std_msgs::Float32MultiArray& motor_commands) {
  for (int motor_id = 0; motor_id < N_MOTOR; motor_id++) { 
    motors[motor_id].Drive(motor_commands.data[motor_id] * safety_limit);
  }
}
ros::Subscriber<std_msgs::Float32MultiArray> sub("motor_command", &commandCb);

int main() {
  nh.initNode();
  nh.subscribe(sub);

  while (1) {
    nh.spinOnce();
    wait_ms(1);
  }
}
