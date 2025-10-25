#ifndef LEG_H
#define LEG_H

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>

class Leg {
  private:
    String _key;
    int _hip;
    int _knee;
    int _hip_max;
    int _hip_min;
    int _knee_max;
    int _knee_min;
    float _theta1;
    float _theta2;
    float thigh;
    float shin;
    Adafruit_PWMServoDriver* _pwm;
    float coordinates[2];
    float x_goal;
    float y_goal;
    float x_step;
    float y_step;
    int step_count;

    bool isEqual(float first, float second, float th);
    void deg2Microsec(float hip_angle, float knee_angle, uint16_t signals[2]);
    void cartesianStep(float x, float y);


  public:
    //Constructor
    Leg() {}
    Leg(String key, int hip, int knee, uint16_t hip_max, uint16_t hip_min, uint16_t knee_max, uint16_t knee_min, Adafruit_PWMServoDriver* pwm);

    //Methods
    void findJoints(float x, float y, float joints[2]);
    void findCartesian(float hip_angle, float knee_angle, float pos[2]);
    void setJoints(float hip_signal, float knee_signal);
    void initLeg(float hip=43, float knee=120);
    void getJoints(float joints[2]);
    void getCoordinates(float coords[2]);
    void cartesian(float x, float y);
    void setGoal(float x, float y);
    byte cartesianInSteps(float x, float y);
};

#endif