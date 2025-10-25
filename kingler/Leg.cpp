#include <math.h>
#include "Leg.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>


Leg::Leg(String key, int hip, int knee, uint16_t hip_max, uint16_t hip_min, uint16_t knee_max, uint16_t knee_min, Adafruit_PWMServoDriver* pwm){
  // define key of leg and coordinates of foot
  _key = key;
  _hip = hip;
  _knee = knee;
  _hip_max = hip_max;
  _hip_min = hip_min;
  _knee_max = knee_max;
  _knee_min = knee_min;
  // thetas in microsecs
  _theta1 = 0.0;
  _theta2 = 0.0;
  thigh = 0.0725;
  shin = 0.1500;
  _pwm = pwm;
  coordinates[0] = thigh+shin;
  coordinates[1] = 0.0;
  x_goal = 0;
  y_goal = 0.15;
  x_step = 0;
  y_step = 0;
  step_count = 0;

}

void Leg::findJoints(float x_cart, float y_cart, float joints_temp[2]){

  float H = sqrt(pow(x_cart,2) + pow(y_cart,2));

  if (H == 0) return;

  float phi1 = acos((pow(thigh,2) + pow(H,2) - pow(shin,2))/(2*thigh*H));
  float phi2 = acos((pow(shin,2) + pow(H,2) - pow(thigh,2))/(2*shin*H));
  float phi3 = atan2(abs(y_cart),x_cart);
 
  if (isnan(phi1)||isnan(phi2)||isnan(phi3)){
    Serial.print(_key);
    Serial.print(": Invalid coordinates (NaN): ");
    Serial.print(x_cart,4);
    Serial.print(", ");
    Serial.println(y_cart,4);
  }
  else{
    // theta1 *-1 to invert
    joints_temp[0] = -180*(phi1-phi3)/PI;
    joints_temp[1] = 180*(phi1+phi2)/PI;
  }
}

void Leg::findCartesian(float hip_angle, float knee_angle, float pos[2]){
  float temp[2];

  temp[0] = thigh*cos(radians(hip_angle)) + shin*cos(radians(hip_angle+knee_angle)); // X coordinates
  temp[1] = thigh*sin(radians(hip_angle)) + shin*sin(radians(hip_angle+knee_angle)); // Y coordinates

  constrain(temp[0],-0.05,0.1);
  constrain(temp[1],0.1,0.2);



  // Check if point is valid (joints within ranges)
  if (isnan(temp[0]) || isnan(temp[1])) {
    Serial.print(_key);
    Serial.println(": Invalid Cartesian result (NaN)");
  }
  else{
    pos[0] = temp[0];
    pos[1] = temp[1];
  }
}

void Leg::initLeg(float hip, float knee){
    Leg::setJoints(hip, knee);
    Leg::findCartesian(hip, knee, coordinates);
    _theta1 = hip;
    _theta2 = knee;
}

void Leg::deg2Microsec(float hip_angle, float knee_angle, uint16_t microsecs[2]){
  if (hip_angle <= 90 && hip_angle >= -25 && knee_angle <= 120 && knee_angle >= 0) {
    // constrain to limits of individual leg
    uint16_t hip_signal = map(round(hip_angle), -25, 90, _hip_max, _hip_min);
    uint16_t knee_signal = map(round(knee_angle), 0, 120, _knee_max, _knee_min);

    microsecs[0] = hip_signal;
    microsecs[1] = knee_signal;
  }
  else{
    // else return current position (no change)
    // constrain to limits of individual leg
    uint16_t hip_signal = map(round(_theta1), -25, 90, _hip_max, _hip_min);
    uint16_t knee_signal = map(round(_theta2), 0, 120, _knee_max, _knee_min);

    microsecs[0] = hip_signal;
    microsecs[1] = knee_signal;
  }
}

void Leg::setJoints(float hip_angle, float knee_angle){ 
  // Serial.print("Angles: ");
  // Serial.print(hip_angle);
  // Serial.print(", ");
  // Serial.println(knee_angle);

  static uint16_t signals[2];
  Leg::deg2Microsec(hip_angle, knee_angle, signals);
  
  signals[0] = constrain(signals[0], _hip_max, _hip_min);
  signals[1] = constrain(signals[1], _knee_max, _knee_min);

  _pwm->writeMicroseconds(_hip, signals[0]);
  _pwm->writeMicroseconds(_knee, signals[1]);

  _theta1 = hip_angle;
  _theta2 = knee_angle;

  Leg::findCartesian(_theta1, _theta2, coordinates);
}

void Leg::getJoints(float joints[2]){
  joints[0] = _theta1;
  joints[1] = _theta2;
}

void Leg::getCoordinates(float coords[2]){
  coords[0] = coordinates[0];
  coords[1] = coordinates[1];
}

void Leg::cartesian(float x, float y){
  float iterations = 20;
  float goal_thold = 0.002;

  float x_diff = x - coordinates[0];
  float y_diff = y - coordinates[1];

  if (abs(x_diff) < goal_thold) x_diff = 0;
  if (abs(y_diff) < goal_thold) y_diff = 0;

  float step_size[] = {x_diff/iterations, y_diff/iterations};
  float c[] = {coordinates[0],coordinates[1]};

  for (int i=0;i<iterations;i++){
    if (Leg::isEqual(x,c[0], goal_thold) && Leg::isEqual(y,c[1], goal_thold)) break;
    c[0] += step_size[0];
    c[1] += step_size[1];
    Leg::cartesianStep(c[0],c[1]);
  }
  Leg::cartesianStep(x,y);

  coordinates[0] = c[0];
  coordinates[1] = c[1];
  Serial.println();
}
bool Leg::isEqual(float first, float second, float th){
  float temp = abs(first-second);
  return (temp<=th);
}

// Move foot in a straight line from current position to new coordinates
void Leg::cartesianStep(float x_step, float y_step){
  static float joint_step[2];
  Leg::findJoints(x_step,y_step,joint_step);
  
  Leg::setJoints(joint_step[0], joint_step[1]);
}

void Leg::setGoal(float x, float y){
  x_goal = x;
  y_goal = y;
  constrain(x_goal, -0.05, 0.1);
  constrain(y_goal, 0.1, 0.2);

  int iterations = 10;
  float goal_thold = 0.002;

  float x_diff = x_goal - coordinates[0];
  float y_diff = y_goal - coordinates[1];

  if (abs(x_diff) < goal_thold) x_diff = 0;
  if (abs(y_diff) < goal_thold) y_diff = 0;

  x_step = x_diff/iterations;
  y_step = y_diff/iterations;
  
}

byte Leg::cartesianInSteps(float x, float y){
  int iterations = 10;
  float goal_thold = 0.002;

  if (step_count == 0) {
    Leg::setGoal(x,y);
  }
  float c[] = { coordinates[0] + x_step, coordinates[1] + y_step };

  if (Leg::isEqual(x,c[0], goal_thold) && Leg::isEqual(y,c[1], goal_thold)){
    step_count = 0;
    return HIGH;
  }
  
  Leg::cartesianStep(c[0],c[1]);

  coordinates[0] = c[0];
  coordinates[1] = c[1];
  step_count += 1;
  if (step_count > iterations) {
    step_count = 0;
    return HIGH;
  }
  return LOW;
}

