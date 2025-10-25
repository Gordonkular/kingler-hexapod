#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Bluepad32.h>
#include "Robot.h"
#include "Leg.h"

ControllerPtr myControllers[BP32_MAX_GAMEPADS];
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
byte tri_gait_state = LOW;
unsigned long previous_millis = 0;
unsigned long current_millis = 0;
int step_interval = 1000; // 0.1s between individual steps
bool standing = false;
bool crouching = false;

int stand_interval = 2000;
bool yellow_direction = false;
bool purple_direction = false;

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
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedController, &onDisconnectedController);

  // "forgetBluetoothKeys()" should be called when the user performs
  // a "device factory reset", or similar.
  // Calling "forgetBluetoothKeys" in setup() just as an example.
  // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
  // But it might also fix some connection / re-connection issues.
  BP32.forgetBluetoothKeys();

  // Enables mouse / touchpad support for gamepads that support them.
  // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
  // - First one: the gamepad
  // - Second one, which is a "virtual device", is a mouse.
  // By default, it is disabled.
  BP32.enableVirtualDevice(false);

  // Kingler code
  delay(1000);
  Serial.println("Boot OK");
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10);
}

void loop() {
  // LOOP CODE FROM BLUEPAD32 EXAMPLE 'Controller'
  // This call fetches all the controllers' data.
  // Call this function in your main loop.
  current_millis = millis();
  bool dataUpdated = BP32.update();
  if (dataUpdated)
      processControllers();

  // The main loop must have some kind of "yield to lower priority task" event.
  // Otherwise, the watchdog will get triggered.
  // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
  // Detailed info here:
  // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

  //     vTaskDelay(1);
  delay(150);
}

void onConnectedController(ControllerPtr ctl) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
            // Additionally, you can get certain gamepad properties like:
            // Model, VID, PID, BTAddr, flags, etc.
            ControllerProperties properties = ctl->getProperties();
            Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                           properties.product_id);
            myControllers[i] = ctl;
            foundEmptySlot = true;
            break;
        }
    }
    if (!foundEmptySlot) {
        Serial.println("CALLBACK: Controller connected, but could not found empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    bool foundController = false;

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
            myControllers[i] = nullptr;
            foundController = true;
            break;
        }
    }

    if (!foundController) {
        Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}

void processControllers() {
    for (auto myController : myControllers) {
        if (myController && myController->isConnected() && myController->hasData()) {
            if (myController->isGamepad()) {
                processGamepad(myController);
            }
            else {
                Serial.println("Unsupported controller");
            }
        }
    }
}


// Process controller inputs
void processGamepad(ControllerPtr ctl) {
    // There are different ways to query whether a button is pressed.
    // By query each button individually:
    //  a(), b(), x(), y(), l1(), etc...
    // X button toggle standing and resting
    if (ctl->buttons() == 0x0001) {
      if (previous_millis - current_millis >= stand_interval){
        if (standing) {
          kingler.rest();
          previous_millis = millis();
          standing = false;
        }
        else {
          kingler.stand();
          previous_millis = millis();
          standing = true;
        }
      }
    }
    // Crouch toggle on square button
    if (ctl->buttons() == 0x0004) {
      if (previous_millis - current_millis >= stand_interval){
        if (crouching) {
          kingler.commandAllJoints(50,56);
          previous_millis = millis();
          crouching = false;
        }
        else {
          kingler.commandAllJoints(5,113);
          previous_millis = millis();
          crouching = true;
        }
      }
    }
    // Triangle, reset to centre
    if (ctl->buttons() == 0x0008) {
      kingler.commandAllJoints(45, 56, 1);
      delay(100);
      kingler.commandAllJoints(50, 56, 1);
      delay(100);
      kingler.commandAllJoints(45, 56, 2);
      delay(100);
      kingler.commandAllJoints(50, 56, 2);
    }

    // Up button is pressed, move towards yellow leg
    if (ctl->dpad() == 0x01) {
      if (yellow_direction) kingler.quadGait(false,false);
      else {
        kingler.quadGait(false, true);
        yellow_direction = true;
      }
    }
    // up button stops being pressed, next time its pressed reset
    if (ctl->dpad() != 0x01){
      yellow_direction = false;
    }
    // Down button is pressed, move towards purple leg
    if (ctl->dpad() == 0x02) {
      if (purple_direction) kingler.quadGait(true,false);
      else {
        kingler.quadGait(true, true);
        purple_direction = true;
      }
    }
    // down button stopped being pressed
    if (ctl->dpad() != 0x02){
      purple_direction = false;
    }
    if (ctl->dpad() == 0x04) {
      kingler.triGait(true);
    }
    if (ctl->dpad() == 0x08) {
      kingler.triGait(false);
    }
}

