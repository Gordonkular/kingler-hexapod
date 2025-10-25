# kingler-hexapod
code base for 6-legged hexapod robot

## License
This project is open source, however it relies on Bluepad32, which relies on the BTStack Library, which is open source only for other open source projects and commercial for closed-source.

## Dependencies:
* Bluepad32 - https://github.com/ricardoquesada/bluepad32
* BTStack - https://github.com/bluekitchen/btstack
* Adafruit-PWM-Servo-Driver-Library - https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library

## Files/Directories:
* kingler directory - Arduino IDE sketch made to be flashed to Kingler, and allow its gait to be controlled wirelessly via PS4 controller
  * flash this sketch to robot to utilise the gait algorithms defined
  * contains:
     * kingler.ino
     * Leg.cpp - Leg object
     * Leg.h
     * Robot.cpp - Robot object
     * Robot.h
* calibrate_servo directory - Arduino IDE sketch made to calibrate the servos and find the PWM signal values for precise angles. Contains the same header and cpp files as kingler, but also the calibrate_servo.ino file.
* inverse_kinematics.mlx - MATLAB live script which finds the reachable and dextrous workspace, and calculates joint torques
* SolidWorks directory - SolidWorks parts and assemblies used to create the finished hexapod 
