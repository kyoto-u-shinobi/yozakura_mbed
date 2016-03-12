#include <vector>
#include "mbed.h"
#include "Dynamixel.h"
#include "mems.h"
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>

std::vector<Dynamixel*> servos;
MX106 *base_yaw = new MX106(p13, p14, 1);
MX106 *base_pitch_master = new MX106(p13, p14, 2);
MX106 *base_pitch_slave = new MX106(p13, p14, 3);   // only for getting information
MX106 *elbow_pitch = new MX106(p13, p14, 4);
MX28 *wrist_yaw = new MX28(p13, p14, 5);

/*
AnalogIn co2(p15);
*/
/*
MEMS thermo_sensors[] = { MEMS(p9, p10),
                          MEMS(p28, p27) };
*/

int minima[] =   { 360,   90,   90,   90,   90};
int maxima[] =   { 360,  270,  270,  359,  270};
float speeds[] = { 0.2,  0.1,  0.1,  0.1,  0.2};
int inits[] =    {   5,  270,  270,   90,  178};
float goals[] =  { 5.0,270.0,270.0, 90.0,178.0};
int command[] =  {   0,    0,    0,    0,    0};
int home_command = 0;

Serial pc(USBTX, USBRX);

ros::NodeHandle nh;


// Callback function
void command_callback(const std_msgs::Int16MultiArray& arm_command) {
  for (int i=0; i<5; i++) {
    command[i]=arm_command.data[i];
  }
  home_command = arm_command.data[5];
}

ros::Subscriber<std_msgs::Int16MultiArray> sub("arm_command", &command_callback);

std_msgs::Float32MultiArray thermo;
ros::Publisher pub_thermal("thermo_data", &thermo);

std_msgs::Float32MultiArray co2;
ros::Publisher pub_co2("co2_data", &co2);

std_msgs::Float32MultiArray position;
ros::Publisher pub_position("joint_angle", &position);


// Get and publish position data
void GetPubPosition() {
  for (int i=0; i<5; i++) {
    position.data[i] = servos[i]->GetPosition();
  }
  pub_position.publish(&position);
}

// Check if any of the dynaimxel is moving
bool AnyDxMoving() {
  for (int i=0; i<5; i++) {
    if (servos[i]->isMoving()) return true;
  }
  return false;
}

// Move dynamixel to home position while avoiding collision
void DxGoHome() {
  wrist_yaw->SetGoal(inits[4]);
  GetPubPosition();
  elbow_pitch->SetGoal(inits[3]+inits[1]-int(position.data[1]));
  if (position.data[0] > -68.0 && position.data[0] < 68.0) {
    base_yaw->SetGoal(inits[0]);
  }
  else if (position.data[0] > -171.0 && position.data[0] < 171.0) {
    base_pitch_master->SetGoal(240);
    base_pitch_slave->SetGoal(240);
    while (1) {
      GetPubPosition();
      elbow_pitch->SetGoal(inits[3]+inits[1]-int(position.data[1]));
      if (position.data[1] < 245.0) {
        break;
      }
      nh.spinOnce();
      wait_ms(10);
    }
  }
  else {
    base_pitch_master->SetGoal(220);
    base_pitch_slave->SetGoal(220);
    while (1) {
      GetPubPosition();
      elbow_pitch->SetGoal(inits[3]+inits[1]-int(position.data[1]));
      if (position.data[1] < 225.0) {
        break;
      }
      nh.spinOnce();
      wait_ms(10);
    }
  }
  base_yaw->SetGoal(inits[0]);
  while (1) {
    GetPubPosition();
    elbow_pitch->SetGoal(inits[3]+inits[1]-int(position.data[1]));
    if (position.data[0] > -68.0 && position.data[0] < 68.0) {
      base_pitch_master->SetGoal(inits[1]);
      base_pitch_slave->SetGoal(inits[2]);
      break;
    }
    nh.spinOnce();
    wait_ms(10);
  }
  base_pitch_master->SetGoal(inits[1]);
  base_pitch_slave->SetGoal(inits[2]);
  while (AnyDxMoving()) {
    GetPubPosition();
    elbow_pitch->SetGoal(inits[3]+inits[1]-int(position.data[1]));
    nh.spinOnce();
    wait_ms(10);
  }
  for (int i=0; i<5; i++) {
    goals[i] = float(inits[i]);
  }
}

// Set angle and angular velocity limits
void DxInitialize() {
  for (int i=0; i<5; i++) {
    servos[i]->SetCWLimit(minima[i]);
    servos[i]->SetCCWLimit(maxima[i]);
    servos[i]->SetCRSpeed(speeds[i]);
  }
}

// Read CO2 data
/*
float GetCO2() {
  return co2.read() * 5000 + 400;  // ppm
}
*/



int main() {
  servos.push_back(base_yaw);
  servos.push_back(base_pitch_master);
  servos.push_back(base_pitch_slave); 
  servos.push_back(elbow_pitch);
  servos.push_back(wrist_yaw);
  
//  float thermo_data[2][16];

  nh.initNode();

  nh.subscribe(sub);

  thermo.data_length = 32;
  thermo.data = (float *)malloc(sizeof(float)*32);
  nh.advertise(pub_thermal);

  nh.advertise(pub_co2);  

  position.data_length = 5;
  position.data = (float *)malloc(sizeof(float)*5);
  nh.advertise(pub_position);

  for (int i=0; i<32; i++) {
    thermo.data[i] = 0.0;
  }
  
  DxInitialize();
  DxGoHome();
  
  // Main loop
  while (1) {

    // Get and publish thermal data
    /*
    for (int i=0; i < 2; i++) {
      thermo_sensors[i].GetTemp(thermo_data[i]);
      for (int j=0; j < 16; j++) {
        thermo.data[i*16+j]=thermo_data[i][j];
      }
    }
    pub_thermal.publish(&thermo);
    */

    // Get and publish co2 data
    /*
    co2.data = GetCO2();
    pub_co2.publish(&co2);
    */
    
    if (home_command) {
      DxGoHome();
    }
    else {
      for (int i=0; i<5; i++) {
        goals[i] += 1.0 * command[i];
        if (i != 0) {
          if (goals[i] > maxima[i]) goals[i] = maxima[i];
          if (goals[i] < minima[i]) goals[i] = minima[i];
        }
        servos[i]->SetGoal(int(goals[i]));
        position.data[i] = servos[i]->GetPosition();
      }
      pub_position.publish(&position);
      
      nh.spinOnce();
      wait_ms(30);
    }
  }
}
