#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include "Leg.h"
#include <Adafruit_PWMServoDriver.h>

class Robot {
  private:
    Adafruit_PWMServoDriver* _pwm;

  public:
    // Six legs 
    Leg yellow_leg, white_leg, black_leg, purple_leg, blue_leg, red_leg;

    Robot(Leg leg1, Leg leg2, Leg leg3, Leg leg4, Leg leg5, Leg leg6, Adafruit_PWMServoDriver* pwm);

    void init();
    void stand();
    void rest();
    Leg* getLeg(String name);
    void commandLeg(String leg_name, float hip, float knee); 
    void commandCart(String leg_name, float x, float y);
    void commandAllJoints(float hip, float knee, int opposites=0);
    void commandAllCart(float x, float y, int opposites=0);
    void triGait(bool dir);
    byte asyncStep(String key, float x, float y);
    void quadGait( bool dir = true, bool reset = false);
};

#endif