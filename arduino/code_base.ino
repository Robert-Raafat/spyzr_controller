#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>


// Pin definitions for motor control
const int motorFL1 = 20;    // Front-left motor positive
const int motorFL2 = 1;    // Front-left motor negative
const int motorFL_Enable = 21;  // Front-left motor enable pin

const int motorFR1 = 6;    // Front-right motor positive
const int motorFR2 = 8;    // Front-right motor negative
const int motorFR_Enable = 5;  // Front-right motor enable pin

const int motorBL1 = 23;    // Back-left motor positive
const int motorBL2 = 0;    // Back-left motor negative
const int motorBL_Enable = 22; // Back-left motor enable pin

const int motorBR1 = 3;   // Back-right motor positive
const int motorBR2 = 2;   // Back-right motor negative
const int motorBR_Enable = 4; // Back-right motor enable pin

double radius = 0.04;
double ly = 0.146;
double lx = 0.11;
ros::NodeHandle  nh;

double w_fl;
double w_fr;
double w_rl;
double w_rr;

double Vx= 0;
double Vy =0;
double Wz = 0;

double Vx_ = 0;
double Vy_ = 0;
double Wz_ = 0;



void commandCallback(const geometry_msgs::Twist& cmd_msg)
{

  Vx = cmd_msg.linear.x;
  Vy = cmd_msg.linear.y;
  Wz = cmd_msg.angular.z;

  w_fl= (1/radius)*(Vx-Vy-(lx+ly)*Wz);
  w_fr= (1/radius)*(Vx+Vy+(lx+ly)*Wz);
  w_rl= (1/radius)*(Vx+Vy-(lx+ly)*Wz);
  w_rr= (1/radius)*(Vx-Vy+(lx+ly)*Wz);

  /*w_fl= (1/radius)*(cmd_msg.linear.x-cmd_msg.linear.y-(lx+ly)*cmd_msg.angular.z);
   w_fr= (1/radius)*(cmd_msg.linear.x+cmd_msg.linear.y+(lx+ly)*cmd_msg.angular.z);
   w_rl= (1/radius)*(cmd_msg.linear.x+cmd_msg.linear.y-(lx+ly)*cmd_msg.angular.z);
   w_rr= (1/radius)*(cmd_msg.linear.x-cmd_msg.linear.y+(lx+ly)*cmd_msg.angular.z);
  */
  Vx_ = (w_fl+w_fr+w_rl+w_rr)*(radius/4); 
  Vy_ = (-w_fl+w_fr+w_rl-w_rr)*(radius/4); 
  Wz_ = (-w_fl+w_fr-w_rl+w_rr)*(radius/(4*(lx+ly))); 
  


  if (Vx_ > 0){
    straightAhead();
  }
  else if (Vy_ > 0 ){
    sideWay();
  }
  else if (Wz_ > 0 ){
    turnRound();
  }
  else if (Wz_ < 0 ){
    turnRoundInv();
  }
  else if (Vx_ < 0 ){
    straightBack();
  }
  else if (Vy_ < 0 ){
    sideWayInv();

  }
  else {
    OFF();
  }

}

ros::Subscriber<geometry_msgs::Twist>sub("spyzr_controller/cmd_vel", &commandCallback );

void setup() {
  // Set motor control pins as outputs
  pinMode(motorFL1, OUTPUT);
  pinMode(motorFL2, OUTPUT);
  pinMode(motorFL_Enable, OUTPUT);

  pinMode(motorFR1, OUTPUT);
  pinMode(motorFR2, OUTPUT);
  pinMode(motorFR_Enable, OUTPUT);

  pinMode(motorBL1, OUTPUT);
  pinMode(motorBL2, OUTPUT);
  pinMode(motorBL_Enable, OUTPUT);

  pinMode(motorBR1, OUTPUT);
  pinMode(motorBR2, OUTPUT);
  pinMode(motorBR_Enable, OUTPUT);
  // Inizialize the ROS node on the Arduino
  nh.initNode();
  // Inform ROS that this node will subscribe to messages on a given topic
  nh.subscribe(sub);
}

void loop() {
  // Move straight
  //straightAhead();
  //delay(2000);  // Wait for 2 seconds

  // Move sideway
  // sideWay();
  // delay(2000);  // Wait for 2 seconds

  // move diagonal
  //diagonal();
  //delay(2000);  // Wait for 2 seconds

  // move concerning
  // concerning();
  // delay(2000);  // Wait for 2 seconds

  // move turnRound
  //turnRound();
  //delay(2000);  // Wait for 2 seconds
  nh.spinOnce();
  delay(1);
  // move turnOfRearAxis
  // turnOfRearAxis();
  //delay(2000);  // Wait for 2 seconds
}

// Function to move the robot forward
void straightAhead() {
  // Set enable pins HIGH to enable the motors
  digitalWrite(motorFL_Enable, HIGH);
  digitalWrite(motorFR_Enable, HIGH);
  digitalWrite(motorBL_Enable, HIGH);
  digitalWrite(motorBR_Enable, HIGH);

  digitalWrite(motorFL1, HIGH);
  digitalWrite(motorFL2, LOW);

  digitalWrite(motorFR1, HIGH);
  digitalWrite(motorFR2, LOW);

  digitalWrite(motorBL1, HIGH);
  digitalWrite(motorBL2, LOW);

  digitalWrite(motorBR1, HIGH);
  digitalWrite(motorBR2, LOW);
}
void straightBack() {
  // Set enable pins HIGH to enable the motors
  digitalWrite(motorFL_Enable, HIGH);
  digitalWrite(motorFR_Enable, HIGH);
  digitalWrite(motorBL_Enable, HIGH);
  digitalWrite(motorBR_Enable, HIGH);

  digitalWrite(motorFL1, LOW);
  digitalWrite(motorFL2, HIGH);

  digitalWrite(motorFR1, LOW);
  digitalWrite(motorFR2, HIGH);

  digitalWrite(motorBL1, LOW);
  digitalWrite(motorBL2, HIGH);

  digitalWrite(motorBR1, LOW);
  digitalWrite(motorBR2, HIGH);
}

void sideWay() {
  // Set enable pins HIGH to enable the motors
  digitalWrite(motorFL_Enable, HIGH);
  digitalWrite(motorFR_Enable, HIGH);
  digitalWrite(motorBL_Enable, HIGH);
  digitalWrite(motorBR_Enable, HIGH);

  digitalWrite(motorFL1, HIGH);
  digitalWrite(motorFL2, LOW);

  digitalWrite(motorFR1, LOW);
  digitalWrite(motorFR2, HIGH);

  digitalWrite(motorBL1, LOW);
  digitalWrite(motorBL2, HIGH);

  digitalWrite(motorBR1, HIGH);
  digitalWrite(motorBR2, LOW);
}
void sideWayInv() {
  // Set enable pins HIGH to enable the motors
  digitalWrite(motorFL_Enable, HIGH);
  digitalWrite(motorFR_Enable, HIGH);
  digitalWrite(motorBL_Enable, HIGH);
  digitalWrite(motorBR_Enable, HIGH);

  digitalWrite(motorFL1, LOW);
  digitalWrite(motorFL2, HIGH);

  digitalWrite(motorFR1, HIGH);
  digitalWrite(motorFR2, LOW);

  digitalWrite(motorBL1, HIGH);
  digitalWrite(motorBL2, LOW);

  digitalWrite(motorBR1, LOW);
  digitalWrite(motorBR2, HIGH);
}

/*
void diagonal() {
  // Set enable pins HIGH to enable the motors
  digitalWrite(motorFL_Enable, HIGH);
  digitalWrite(motorFR_Enable, LOW);
  digitalWrite(motorBL_Enable, LOW);
  digitalWrite(motorBR_Enable, HIGH);

  digitalWrite(motorFL1, HIGH);
  digitalWrite(motorFL2, LOW);

  digitalWrite(motorFR1, LOW);
  digitalWrite(motorFR2, HIGH);

  digitalWrite(motorBL1, LOW);
  digitalWrite(motorBL2, HIGH);

  digitalWrite(motorBR1, HIGH);
  digitalWrite(motorBR2, LOW);
}

void concerning() {
  // Set enable pins HIGH to enable the motors
  digitalWrite(motorFL_Enable, HIGH);
  digitalWrite(motorFR_Enable, LOW);
  digitalWrite(motorBL_Enable, HIGH);
  digitalWrite(motorBR_Enable, LOW);

  digitalWrite(motorFL1, HIGH);
  digitalWrite(motorFL2, LOW);

  digitalWrite(motorFR1, LOW);
  digitalWrite(motorFR2, HIGH);

  digitalWrite(motorBL1, HIGH);
  digitalWrite(motorBL2, LOW);

  digitalWrite(motorBR1, LOW);
  digitalWrite(motorBR2, HIGH);
}
*/
void turnRound() {
  // Set enable pins HIGH to enable the motors
  digitalWrite(motorFL_Enable, HIGH);
  digitalWrite(motorFR_Enable, HIGH);
  digitalWrite(motorBL_Enable, HIGH);
  digitalWrite(motorBR_Enable, HIGH);

  digitalWrite(motorFL1, HIGH);
  digitalWrite(motorFL2, LOW);

  digitalWrite(motorFR1, LOW);
  digitalWrite(motorFR2, HIGH);

  digitalWrite(motorBL1, HIGH);
  digitalWrite(motorBL2, LOW);

  digitalWrite(motorBR1, LOW);
  digitalWrite(motorBR2, HIGH);
}
void turnRoundInv() {
  // Set enable pins HIGH to enable the motors
  digitalWrite(motorFL_Enable, HIGH);
  digitalWrite(motorFR_Enable, HIGH);
  digitalWrite(motorBL_Enable, HIGH);
  digitalWrite(motorBR_Enable, HIGH);

  digitalWrite(motorFL1, LOW);
  digitalWrite(motorFL2, HIGH);

  digitalWrite(motorFR1, HIGH);
  digitalWrite(motorFR2, LOW);

  digitalWrite(motorBL1, LOW);
  digitalWrite(motorBL2, HIGH);

  digitalWrite(motorBR1, HIGH);
  digitalWrite(motorBR2, LOW);
}

void OFF() {
  // Set enable pins HIGH to enable the motors
  digitalWrite(motorFL_Enable, LOW);
  digitalWrite(motorFR_Enable, LOW);
  digitalWrite(motorBL_Enable, LOW);
  digitalWrite(motorBR_Enable, LOW);
}
/*
void turnOfRearAxis() {
  // Set enable pins HIGH to enable the motors
  digitalWrite(motorFL_Enable, HIGH);
  digitalWrite(motorFR_Enable, HIGH);
  digitalWrite(motorBL_Enable, LOW);
  digitalWrite(motorBR_Enable, LOW);

  digitalWrite(motorFL1, HIGH);
  digitalWrite(motorFL2, LOW);

  digitalWrite(motorFR1, HIGH);
  digitalWrite(motorFR2, LOW);

  digitalWrite(motorBL1, LOW);
  digitalWrite(motorBL2, HIGH);

  digitalWrite(motorBR1, LOW);
  digitalWrite(motorBR2, HIGH);
}
*/
