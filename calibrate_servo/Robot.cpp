#include <math.h>
#include "Leg.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "Robot.h"

#define THIGH 0.725
#define SHIN 0.15


Robot::Robot(Leg leg1, Leg leg2, Leg leg3, Leg leg4, Leg leg5, Leg leg6, Adafruit_PWMServoDriver* pwm){
  _pwm = pwm;
  red_leg = leg1;
  yellow_leg = leg2;
  black_leg = leg3;
  white_leg = leg4;
  purple_leg = leg5;
  blue_leg = leg6;
}

void Robot::init(){
  yellow_leg.initLeg(90,100);
  white_leg.initLeg();
  black_leg.initLeg();
  purple_leg.initLeg(90,100);
  blue_leg.initLeg();
  red_leg.initLeg();
}

// Robot stand up sequence from legs outstretched
void Robot::stand(){
  Robot::commandAllJoints(60, 100);
  Robot::commandAllJoints(60, 90);

  Robot::commandLeg("yellow",5,113);
  Robot::commandLeg("white",5,113);
  Robot::commandLeg("blue",5,113);
  Robot::commandLeg("purple",5,113);
  Robot::commandLeg("black",5,113);
  Robot::commandLeg("red",5,113);


  Robot::commandAllJoints(13,103);

  Robot::commandAllJoints(50,56);



}

// Robot collapse into resting position
void Robot::rest(){
  Robot::commandAllJoints(50, 55);
  Robot::commandAllJoints(60, 70);
  Robot::commandAllJoints(60, 100);
  Robot::init();
}

// Robot pack away
void Robot::packDown(){

}

Leg* Robot::getLeg(String leg_name){
  if (leg_name == "purple") return &purple_leg;
  if (leg_name == "yellow") return &yellow_leg;
  if (leg_name =="black") return &black_leg;
  if (leg_name == "white") return &white_leg;
  if (leg_name == "red") return &red_leg;
  if (leg_name == "blue") return &blue_leg;
}

void Robot::commandLeg(String leg_name, float hip, float knee) {
  Leg* leg = Robot::getLeg(leg_name);
  if (leg == nullptr) return;

  float pos[2];
  leg->getJoints(pos);

  float hipStart = pos[0];
  float kneeStart = pos[1];

  int steps = 30;       // number of interpolation steps
  int stepDelay = 10;   // ms between steps (~0.3 sec total motion)

  for (int i = 0; i <= steps; i++) {
    float next_hip = hipStart + (hip - hipStart) * (i / (float)steps);
    float next_knee = kneeStart + (knee - kneeStart) * (i / (float)steps);

    leg->setJoints(next_hip, next_knee);
    delay(stepDelay);
  }
}

void Robot::commandCart(String leg_name, float x, float y){
  Leg* leg = Robot::getLeg(leg_name);
  if (leg == nullptr) return;

  leg->cartesian(x, y);
}

void Robot::commandAllJoints(float hip, float knee, int opposites){
  switch(opposites){
    case 0:

      yellow_leg.setJoints(hip, knee);
      purple_leg.setJoints(hip, knee);

      blue_leg.setJoints(hip, knee);
      red_leg.setJoints(hip, knee);
      
      black_leg.setJoints(hip, knee);
      white_leg.setJoints(hip, knee);
      delay(200);
      break;
    case 1:
      yellow_leg.setJoints(hip, knee);
      blue_leg.setJoints(hip, knee);
      white_leg.setJoints(hip, knee);
      
      break;
    case 2:
      purple_leg.setJoints(hip, knee);
      black_leg.setJoints(hip, knee);
      red_leg.setJoints(hip, knee);

      break;
  }
}


void Robot::commandAllCart(float x, float y, int opposites){
  switch(opposites){
    case 0:
      blue_leg.cartesian(x, y);
      red_leg.cartesian(x, y);
      
      black_leg.cartesian(x, y);
      white_leg.cartesian(x, y);

      yellow_leg.cartesian(x, y);
      purple_leg.cartesian(x, y);
      
      break;
    case 1:
      yellow_leg.cartesian(x, y);
      blue_leg.cartesian(x, y);
      white_leg.cartesian(x, y);
      
      break;
    case 2:
      purple_leg.cartesian(x, y);
      black_leg.cartesian(x, y);
      red_leg.cartesian(x, y);

      break;
  }
}
byte Robot::asyncStep(String key, float x, float y) {
  // Serial.print("asyncStep: ");
  // Serial.print(key);
  // Serial.print(": ");
  // Serial.print(x);
  // Serial.print(", ");
  // Serial.println(y);


  Leg *leg = Robot::getLeg(key);
  return leg->cartesianInSteps(x,y);
}

void Robot::triGait(bool dir) {
  unsigned long current_millis = millis();
  static unsigned long previous_millis = 0;
  Serial.println(current_millis);
  Serial.println(previous_millis);

  float fwd = 0.1;
  float back = -0.05;
  float up = 0.14;
  float down = 0.19;
  float step_interval = 1000;
  static bool tri_gait_state = LOW;
  if (dir == LOW) {
    fwd = -0.05;
    back = 0.1;
  };
  if (tri_gait_state == LOW){
    if (current_millis - previous_millis >= step_interval){
      tri_gait_state = HIGH;
      previous_millis = current_millis;
      bool nextStage = false;
      bool part1 = false;
      bool part2 = false;
      bool part3 = false;

      while (!nextStage) {
        if (!part1) {
          if (Robot::asyncStep("purple", back, up)==HIGH) part1 = true;
        }
        if (!part2) {
          if (Robot::asyncStep("red", fwd, up)==HIGH) part2 = true;
        }
        if (!part3) {
          if (Robot::asyncStep("black", fwd, up)==HIGH) part3 = true;
        }
        if (part1 && part2 && part3) nextStage = true;
      }
      part1 = false;
      part2 = false;
      part3 = false;
      nextStage = false;
      delay(100);

      // second set move back
      
      while (!nextStage) {
        if (!part1) {
          if (Robot::asyncStep("yellow", back, down)==HIGH) part1 = true;
        }
        if (!part2) {
          if (Robot::asyncStep("blue", fwd, down)==HIGH) part2 = true;
        }
        if (!part3) {
          if (Robot::asyncStep("white", fwd, down)==HIGH) part3 = true;
        }
        if (part1 && part2 && part3) nextStage = true;
      }
      part1 = false;
      part2 = false;
      part3 = false;
      nextStage = false;
      // // first set move down
      delay(100);

      while (!nextStage) {
        if (!part1) {
          if (Robot::asyncStep("purple", back, down+0.01)==HIGH) part1 = true;
        }
        if (!part2) {
          if (Robot::asyncStep("red", fwd, down+0.01)==HIGH) part2 = true;
        }
        if (!part3) {
          if (Robot::asyncStep("black", fwd, down+0.01)==HIGH) part3 = true;
        }
        if (part1 && part2 && part3) nextStage = true;
      }
    }
  }
  else{
    if (current_millis - previous_millis >= step_interval){
      tri_gait_state = LOW;
      previous_millis = current_millis;
      bool nextStage = false;
      bool part1 = false;
      bool part2 = false;
      bool part3 = false;

      while (!nextStage) {
        if (!part1) {
          if (Robot::asyncStep("yellow", fwd, up)==HIGH) part1 = true;
        }
        if (!part2) {
          if (Robot::asyncStep("blue", back, up)==HIGH) part2 = true;
        }
        if (!part3) {
          if (Robot::asyncStep("white", back, up)==HIGH) part3 = true;
        }
        if (part1 && part2 && part3) nextStage = true;
      }
      part1 = false;
      part2 = false;
      part3 = false;
      nextStage = false;
      delay(100);
      
      // second set move back
      
      while (!nextStage) {
        if (!part1) {
          if (Robot::asyncStep("purple", fwd, down)==HIGH) part1 = true;
        }
        if (!part2) {
          if (Robot::asyncStep("red", back, down)==HIGH) part2 = true;
        }
        if (!part3) {
          if (Robot::asyncStep("black", back, down)==HIGH) part3 = true;
        }
        if (part1 && part2 && part3) nextStage = true;
      }
      part1 = false;
      part2 = false;
      part3 = false;
      nextStage = false;
      delay(100);

      // // first set move down
      while (!nextStage) {
        if (!part1) {
          if (Robot::asyncStep("yellow", fwd, down+0.01)==HIGH) part1 = true;
        }
        if (!part2) {
          if (Robot::asyncStep("blue", back, down+0.01)==HIGH) part2 = true;
        }
        if (!part3) {
          if (Robot::asyncStep("white", back, down+0.01)==HIGH) part3 = true;
        }
        if (part1 && part2 && part3) nextStage = true;
      }
    }
  }
  previous_millis = millis();
}
