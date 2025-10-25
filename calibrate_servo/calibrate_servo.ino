/*************************************************** 
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 8 servos, one after the other on the
  first 8 pins of the PCA9685

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815
  
  These drivers use I2C to communicate, 2 pins are required to  
  interface.

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "Leg.h"
#include "Robot.h"
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define SERVOBOTTOM 500 // apprx zero point
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150

#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define YLW_HIP_TOP  766 // apprx -25 degrees for hip
#define YLW_HIP_BOT  2063 // apprx 90 degrees in microsecs for hip
#define YLW_KNEE_TOP 852
#define YLW_KNEE_BOT 2113

#define WHT_HIP_TOP  771 // apprx -25 degrees for hip
#define WHT_HIP_BOT  1994 // apprx 90 degrees in microsecs for hip
#define WHT_KNEE_TOP 829
#define WHT_KNEE_BOT 2089

#define BLK_HIP_TOP  747 // apprx -25 degrees for hip
#define BLK_HIP_BOT  1979 // apprx 90 degrees in microsecs for hip
#define BLK_KNEE_TOP 889
#define BLK_KNEE_BOT 2150

#define PUR_HIP_TOP  861 // apprx -25 degrees for hip
#define PUR_HIP_BOT  2101 // apprx 90 degrees in microsecs for hip
#define PUR_KNEE_TOP 869 // apprx 0 deg at knee
#define PUR_KNEE_BOT 2129 // apprx 120 deg at knee

#define BLU_HIP_TOP  711 // apprx -25 degrees for hip
#define BLU_HIP_BOT  2013 // apprx 90 degrees in microsecs for hip
#define BLU_KNEE_TOP 860 // apprx 0 deg at knee
#define BLU_KNEE_BOT 2110 // apprx 120 deg at knee

#define RED_HIP_TOP  709 // apprx -25 degrees for hip
#define RED_HIP_BOT  1960 // apprx 90 degrees in microsecs for hip
#define RED_KNEE_TOP 770 // apprx 0 deg at knee
#define RED_KNEE_BOT 2020 // apprx 120 deg at knee

#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// YLW add 5
// WHITE add 2 deg to hip, add 1 degrees to knee end
// Bpack add 1
//Purp add 2

// our servo # counter
uint8_t servonum = 0;
// Yellow Shin, 0 deg @ 800, end @ 2100
// White Shin, 0 deg @ 850, end @ 2150
// Black Shin, 0 deg @ 900, end @ 2200
unsigned long current_millis = 0;
Leg red_leg("red", 0,1, RED_HIP_TOP, RED_HIP_BOT, RED_KNEE_TOP, RED_KNEE_BOT, &pwm); 
Leg yellow_leg("yellow", 2, 3, YLW_HIP_TOP, YLW_HIP_BOT, YLW_KNEE_TOP, YLW_KNEE_BOT, &pwm);
Leg black_leg("black", 6,7, BLK_HIP_TOP, BLK_HIP_BOT, BLK_KNEE_TOP, BLK_KNEE_BOT, &pwm);
Leg white_leg("white", 8,9, WHT_HIP_TOP, WHT_HIP_BOT, WHT_KNEE_TOP, WHT_KNEE_BOT, &pwm);
Leg purple_leg("purple", 10,11, PUR_HIP_TOP, PUR_HIP_BOT, PUR_KNEE_TOP, PUR_KNEE_BOT, &pwm); 
Leg blue_leg("blue", 12,13, BLU_HIP_TOP, BLU_HIP_BOT, BLU_KNEE_TOP, BLU_KNEE_BOT, &pwm);

Robot kingler(red_leg, yellow_leg, black_leg, white_leg, purple_leg, blue_leg, &pwm);

float thigh = 0.15;
float shin = 0.725;
float hipPulse = 0.0;  // start neutral
float kneePulse = 0.0;  // start neutral


void setup() {
  Serial.begin(9600);
  delay(1000);
  Serial.println("Boot OK");
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  // (std::string key, int hip, int knee, int hip_max, int hip_min, int knee_max, int knee_min, Adafruit_PWMServoDriver* pwm)

  delay(100);
  delay(100);
}

void loop() {
  current_millis = millis();
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'q') hipPulse += 5;
      if (c == 'w') hipPulse -=5;
      if (c == 'e') kingler.rest();
      if (c == 'r') kingler.stand();
      if (c == 'z') kingler.commandAllJoints(50,50,1);
      if (c == 'x') kingler.commandAllJoints(50,50,2);
      if (c == 'a') kneePulse += 5;
      if (c == 's') kneePulse -= 5;
      if (c == 'd') kingler.commandLeg("yellow", hipPulse, kneePulse);
      Serial.print("Angles: ");
      Serial.print(hipPulse);
      Serial.print(", ");
      Serial.println(kneePulse);

      // HIPS
      // Blue: 80 @ 1900, 22 @ 750
      // Black: 85 @ 1925, 25 @ 775
      // Purple: 82 @ 2015, 26 @ 865
      // Yellow: 80 @ 1950, 26 @ 800
      // Red: 83 @ 1885, 24 @ 735
      // White: 83 @ 1920, 24 @ 770

      // Blue: 997, 2
      // Black: 1036, 2
      // Purple: 1142, 1
      // Yellow: 1082, 3
      // Red: 992, 1
      // White: 1027, -1


      // KNEES
      // Blue: 1 @ 870, 121 @ 2120
      // Black: 3 @ 900, 120 @ 2150
      // Purple: 2 @ 890, 121 @ 2140
      // Yellow: 2 @ 820, 119 @ 2070
      // Red: 1 @ 780, 121 @ 2030
      // White: 2 @ 850, 121 @ 2100
    }
}




