/******************************************************************************************************************
 * Copyright © 2018. Confidential and proprietary information of Astrobotic Technology, Inc. All Rights Reserved.
 * 
 * AUTHOR: Avery E. Horvath 
 * email:  avery.horvath1@gmail.com
 ******************************************************************************************************************/
 
#include <PS2X_lib.h>
#include <Omni3WD.h> 
#include <MotorWheel.h>
#include <PID_Beta6.h>

#define DEBUG 1

/************************************************************************************************************/
/* Pins for PS/2 remot control
PS/2 pin A2  - Data 
PS/2 pin A3  - COMMAND
PS/2 pin A4  - ATT
PS/2 pin A5  - Clock
PS/2 pin 5V  - 5V
PS/2 pin GND - Ground
*/
#define DAT_PIN  A2
#define CMD_PIN  A3
#define ATT_PIN  A4
#define CLK_PIN  A5

//Initialize 3 Motors
irqISR(irq1,isr1);               
MotorWheel wheel1(9,8,6,7,&irq1); // This will create a MotorWheel objectt called wheel1
                                  // Motor PWM:Pin9, DIR:Pin8, Encoder A:Pin6, B:Pin7
irqISR(irq2,isr2);
MotorWheel wheel2(10,11,14,15,&irq2);

irqISR(irq3,isr3);
MotorWheel wheel3(3,2,4,5,&irq3); 

Omni3WD Omni(&wheel1,&wheel2,&wheel3);

// Initialize PS/2 remote control
PS2X ps2x;

unsigned int PlyStnRStickUpDn = 0;  // Value read off the PS2 Right Stick up/down.
unsigned int ps2RStickLtRt = 0;  // Value read off the PS2 Right Stick left/right
unsigned int ps2LStickUpDn = 0;  // Value read off the PS2 Left Stick up/down
unsigned int ps2LStickLtRt = 0;  // Value read off the PS2 Left Stick left/right

int ps2ControllerError = 0; // Return value to check status of PS/2 remote
byte ps2ControllerType = 0; // PS/2 controller ps2ControllerType

volatile boolean PRECISION_MODE;

unsigned int speedMMPS = 130;
const unsigned int ZeroPosLtRt = 128;
const unsigned int ZeroPosUpDn = 127;
const unsigned int MaxSpeed = 255;
const unsigned int MinSpeed = 0;
const unsigned int speedIncrement = 20;
const unsigned int rotateSpeed = 100;
const unsigned int stickThreshold1 = 40; // For on-axis movement
const unsigned int stickThreshold2 = 40; // For diagonal movement 
const unsigned int stickThreshold3 = 10; // For rotations

/************************************************************************************************************/

void setup() {
  // Initialize seriol connection
#ifdef DEBUG
  Serial.begin(9600);
#endif

  // Set up timers for motors
  TCCR1B=TCCR1B&0xf8|0x01;        // Timer1 Pin9,Pin10 PWM 31250Hz
  TCCR2B=TCCR2B&0xf8|0x01;        // Timer2 Pin3,Pin11 PWM 31250Hz
  
  Omni.PIDEnable(0.26,0.02,0,10); // Enable PID 
  Omni.setCarStop();              // Set speed to zero at start

  // Configure pins for PS/2 remote
  pinMode(DAT_PIN, INPUT);   // Has to be input
  pinMode(CMD_PIN, OUTPUT);  // Has to be output
  pinMode(ATT_PIN, OUTPUT);  // Has to be output
  pinMode(CLK_PIN, OUTPUT);  // Has to be output

  // set up pins and settings: GamePad(clock, command, attention, data, Pressures, Rumble)
  ps2ControllerError = ps2x.config_gamepad(CLK_PIN,CMD_PIN,ATT_PIN,DAT_PIN, false, false);
  
#ifdef DEBUG
  // Check ps2ControllerError code from remote control
  switch (ps2ControllerError) {
  case 0: 
    Serial.println("Found Controller, configured successful");
    break;
  case 1: 
    Serial.println("No controller found, check wiring.");
    break;
  case 2: 
    Serial.println("Controller found but not accepting commands, please refer to the program author's website debugging instructions.");
    break;
  case 3:
    Serial.println("Controller refusing to enter pressures mode, may not support it.");
    break;
  }

  // Check PS/2 controller ps2ControllerType
  ps2ControllerType = ps2x.readType(); 
  switch(ps2ControllerType) {
  case 0:
    Serial.println("Unknown Controller ps2ControllerType");
    break;
  case 1:
    Serial.println("DualShock Controller Found");
    break;
  }
#endif
  
  PRECISION_MODE = false; // Must be defined in setup
  
  delay(2000); // Two second delay to allow controller to fully initialize.
}

/************************************************************************************************************/

void loop() {
 
  if (ps2ControllerError == 1) {
    return; //skip loop if no controller found
  }
 
  ps2x.read_gamepad(); // This needs to be called at least once a second

  if (ps2x.ButtonPressed(PSB_GREEN)) { // Triangle pressed
    PRECISION_MODE = !PRECISION_MODE;  // Toggle the staggered movement mode
  }
  
  //Analog Stick readings
  ps2LStickUpDn = ps2x.Analog(PSS_LY); // Left Stick Up and Down,     0 --> 255 & 127 at zero position
  ps2LStickLtRt = ps2x.Analog(PSS_LX); // Left Stick Left and Right,  0 --> 255 & 127 at zero position
  ps2RStickLtRt = ps2x.Analog(PSS_RX); // Right Stick Left and Right, 0 --> 255 & 128 at zero position
  
#ifdef DEBUG
  Serial.print("ps2LStickUpDn: ");
  Serial.print(ps2LStickUpDn);
  Serial.print(", ps2LStickLtRt: ");
  Serial.print(ps2LStickLtRt);
  Serial.print(", ps2RStickLtRt: ");
  Serial.println(ps2RStickLtRt);
#endif

  // Increase or decrease robot speed
  if (ps2x.ButtonPressed(PSB_R2)) { // Right trigger
    speedMMPS = (speedMMPS > MaxSpeed-speedIncrement) ? MaxSpeed : speedMMPS+speedIncrement;
  }
  
  if (ps2x.ButtonPressed(PSB_L2)) { // Left trigger
    speedMMPS = (speedMMPS < MinSpeed+speedIncrement) ? MinSpeed : speedMMPS-speedIncrement;
  }

  // Left JoyStick --> Direction
  // +/-threshold from zero position so that the joystick doesn't have to point 
  // exactly in the desired direction of the robot. This makes the movement
  // smoother and makes the robot easier to control.
  if ((ps2LStickLtRt > ZeroPosLtRt + stickThreshold2) && (ps2LStickUpDn > ZeroPosUpDn + stickThreshold2)) {
    southEastDir(speedMMPS);
  }
  else if ((ps2LStickLtRt < ZeroPosLtRt - stickThreshold2) && (ps2LStickUpDn < ZeroPosUpDn - stickThreshold2)) { 
    northWestDir(speedMMPS);
  } 
  else if ((ps2LStickLtRt > ZeroPosLtRt + stickThreshold2) && (ps2LStickUpDn < ZeroPosUpDn - stickThreshold2)) {
    northEastDir(speedMMPS);
  }
  else if ((ps2LStickLtRt < ZeroPosLtRt - stickThreshold2) && (ps2LStickUpDn > ZeroPosUpDn + stickThreshold2)) {
    southWestDir(speedMMPS);
  }
  else if (ps2LStickLtRt > ZeroPosLtRt + stickThreshold1) {
    moveRight(speedMMPS);
  }
  else if (ps2LStickLtRt < ZeroPosLtRt - stickThreshold1) {
    moveLeft(speedMMPS);
  }
  else if (ps2LStickUpDn < ZeroPosUpDn - stickThreshold1) {
    forwardDir(speedMMPS);
  }
  else if (ps2LStickUpDn > ZeroPosUpDn + stickThreshold1) {
    backwardsDir(speedMMPS);
  }
   
  // Right Joystick --> Rotation - only if left stick is in zero position
  else if (ps2RStickLtRt > ZeroPosLtRt + stickThreshold3) { 
    rotateRight();
  }
  else if (ps2RStickLtRt < ZeroPosLtRt - stickThreshold3) {
    rotateLeft();
  }
  else if (Omni.getCarStat() != Omni3WD::STAT_STOP) { // If car is not already stopped, slowly come to a stop
    Omni.setCarSlow2Stop(100);
  }
  else {
    Omni.setCarStop(); // If the code gets to this statment, the robot is already stopped. This statment makes sure the robot stays stopped
  }
  

  Omni.PIDRegulate(); // Must be called once per loop
  
  delay(15); // alleviates the small twitches from the wheels
}

/************************************************************************************************************/

void forwardDir(unsigned int forwardSpeed) {    
  if(Omni.getCarStat() != Omni3WD::STAT_ADVANCE) {
    Omni.setCarSlow2Stop(100);
  }
  if (PRECISION_MODE) {
    Omni.setCarAdvance(0);
    Omni.setCarSpeedMMPS(forwardSpeed);
  }
  else {
    Omni.setCarAdvance(forwardSpeed);
  }
}

void backwardsDir(unsigned int backwardSpeed) {    
  if(Omni.getCarStat()!=Omni3WD::STAT_BACKOFF) {
    Omni.setCarSlow2Stop(100);
  }
   
  if (PRECISION_MODE) {
    Omni.setCarBackoff(0);
    Omni.setCarSpeedMMPS(backwardSpeed);
  }
  else {
    Omni.setCarBackoff(backwardSpeed);
  }
}

void moveRight(unsigned int rightSpeed) {
  if(Omni.getCarStat()!=Omni3WD::STAT_RIGHT) {
    Omni.setCarSlow2Stop(100);
  }
   
  if (PRECISION_MODE) {
    Omni.setCarRight(0);
    Omni.setCarSpeedMMPS(rightSpeed);
  }
  else {
    Omni.setCarRight(rightSpeed);
  }
}


void moveLeft(unsigned int leftSpeed) {
  if(Omni.getCarStat()!=Omni3WD::STAT_LEFT) {
    Omni.setCarSlow2Stop(100);
  }
   
  if (PRECISION_MODE) {
    Omni.setCarLeft(0);
    Omni.setCarSpeedMMPS(leftSpeed);
  }
  else {
    Omni.setCarLeft(leftSpeed);
  }
}

void northEastDir(unsigned int northEastSpeed) {
  Omni.wheelBackSetSpeedMMPS(northEastSpeed,DIR_BACKOFF);
  Omni.wheelRightSetSpeedMMPS(0,DIR_BACKOFF);
  Omni.wheelLeftSetSpeedMMPS(northEastSpeed,DIR_ADVANCE);
  delay(15);
}

void southEastDir(unsigned int southEastSpeed) {
  Omni.wheelBackSetSpeedMMPS(southEastSpeed,DIR_BACKOFF);
  Omni.wheelRightSetSpeedMMPS(southEastSpeed,DIR_ADVANCE);
  Omni.wheelLeftSetSpeedMMPS(0,DIR_BACKOFF);
  delay(15);
}

void northWestDir(unsigned int northWestSpeed) {
  Omni.wheelBackSetSpeedMMPS(northWestSpeed,DIR_ADVANCE);
  Omni.wheelRightSetSpeedMMPS(northWestSpeed,DIR_BACKOFF);  
  Omni.wheelLeftSetSpeedMMPS(0,DIR_BACKOFF);  
  delay(15);
}

void southWestDir(unsigned int southWestSpeed) {
  Omni.wheelBackSetSpeedMMPS(southWestSpeed,DIR_ADVANCE);
  Omni.wheelRightSetSpeedMMPS(0,DIR_BACKOFF);
  Omni.wheelLeftSetSpeedMMPS(southWestSpeed,DIR_BACKOFF);
  delay(15);
}


void rotateRight() {
  if(Omni.getCarStat()!=Omni3WD::STAT_ROTATERIGHT) { // Makes transition of directions smoother. Note: doing same thing for
    Omni.setCarSlow2Stop(100);                       // rotateLeft causes a staggered movement in the wheels.
  } 
  Omni.setCarRotateRight(rotateSpeed);
}

void rotateLeft() {
  if(Omni.getCarStat()!=Omni3WD::STAT_ROTATELEFT) { 
    Omni.setCarSlow2Stop(100);
  }
  Omni.setCarRotateLeft(rotateSpeed);
}


