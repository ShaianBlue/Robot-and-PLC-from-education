/***************************************************************************
*   Copyright (C) 2024 by DTU                             *
*   jcan@dtu.dk                                                    *
*
*   Base Teensy firmware
*   build for Teensy 4.1,
*   intended for digital control course
* 
* The MIT License (MIT)  https://mit-license.org/
* 
* Permission is hereby granted, free of charge, to any person obtaining a copy of this software
* and associated documentation files (the “Software”), to deal in the Software without restriction, 
* including without limitation the rights to use, copy, modify, merge, publish, distribute, 
* sublicense, and/or sell copies of the Software, and to permit persons to whom the Software 
* is furnished to do so, subject to the following conditions:
* 
* The above copyright notice and this permission notice shall be included in all copies 
* or substantial portions of the Software.
* 
* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
* INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR 
* PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE 
* FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, 
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN 
* THE SOFTWARE. */

void printLog();

#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include <IntervalTimer.h>
// Support functions
#include "src/udisplay.h"
#include "src/urobot.h"
#include "src/uusb.h"
#include "src/umotor.h"
#include "src/uimu2.h"
#include "src/uencoder.h"

/**
 * Global variables */
// Sample time can not go lower than 300us
const uint32_t sampleTimeUs = 1000; // desired sample time in us
// Robot configuration
const float gear = 9.6;
const float wheelRadius = 0.0315; // (m)
const float r2m = wheelRadius / gear;

// ////////////////////////////////////////
// define a structure for log elements
typedef struct
{
  float time;
  int state;
  float desiredValue[2];
  float motorVel[2];
  float motorVolt[2];
  int32_t encoder[2];
  float motorCurrent[2];
  float battery;
  float gyro[3];
  float position[2];
  float heading;
  float velocity[2];
  float turnrate;
} LogData;
//
// allocate memory for data log in memory block 2 available only through malloc/new
// (specific for this micro processor, a Teensy 4)
const int logsMax = 400000 / sizeof(LogData); // allocates 400kBytes
LogData * logs = (LogData*)malloc(logsMax * sizeof(LogData));
int logsCnt = 0; // next log entry to use / i.e. number of used log entries

void printLog()
{
  LogData * d = logs;
  Serial.print("% Basebot log for ");
  Serial.println(robot.getRobotName());
  // hard-coded settings
  Serial.print("% Sample time ");  Serial.print(sampleTimeUs);
  Serial.print(" us, Gear ");  Serial.print(gear);
  Serial.print(", Wheels ");  Serial.print(wheelRadius);
  Serial.print(" m, PPR ");  Serial.println(encoder.pulsPerRev);
  Serial.print("% maximum log samples ");  Serial.print(logsMax);
  Serial.print(" each sized ");  Serial.print(sizeof(LogData)); Serial.println(" bytes");
  // Columns
  Serial.println("% 1 Time (s)");
  Serial.println("% 2 State");
  Serial.println("% 3 Velocity desired value (m/s)");
  Serial.println("% 4 Turnrate desired value (rad/s)");
  Serial.println("% 5-6 Motor voltage (left,right) (V)");
  Serial.println("% 7-8 Encoder count (left, right)");
  Serial.println("% 9-10 Motor angular velocity (left, right) (rad/s)");
  Serial.println("% 11 Battery voltage (V)");
  Serial.println("% 12-13 Motor current (left, right) (A)");
  Serial.println("% 14-16 Gyro data (x, y, z) (deg/s)");
  Serial.println("% 17-18 Estimated position (x, y) (m)");
  Serial.println("% 19 Estimated heading (rad)");
  Serial.println("% 20-21 Estimated velocity (x, y) (m/s)");
  Serial.println("% 22 Estimated turnrate (rad/s)");
  

  for (int i = 0; i < logsCnt; i++)
  {
    // Serial.print(" ");
    Serial.print(d->time,4);    // 1
    Serial.print(" ");    Serial.print(d->state);          // 2
    Serial.print(" ");    Serial.print(d->desiredValue[0],2); // 3
    Serial.print(" ");    Serial.print(d->desiredValue[1],2); // 5
    Serial.print(" ");    Serial.print(d->motorVolt[0],2); // 5 (V)
    Serial.print(" ");    Serial.print(d->motorVolt[1],2); // 6 (V)
    Serial.print(" ");    Serial.print(d->encoder[0]);     // 7
    Serial.print(" ");    Serial.print(d->encoder[1]);     // 8
    Serial.print(" ");    Serial.print(d->motorVel[0],2);  // 9 (rad/s)
    Serial.print(" ");    Serial.print(d->motorVel[1],2);  // 10 (rad/s)
    Serial.print(" ");    Serial.print(d->battery,2);      // 11 (V)
    Serial.print(" ");    Serial.print(d->motorCurrent[0],3);  // 12 (V)
    Serial.print(" ");    Serial.print(d->motorCurrent[1],3);  // 13 (V)
    Serial.print(" ");    Serial.print(d->gyro[0],1);     // 14 (rad/s)
    Serial.print(" ");    Serial.print(d->gyro[1],1);     // 15 (rad/s)
    Serial.print(" ");    Serial.print(d->gyro[2],1);     // 16 (deg/s)
    Serial.print(" ");    Serial.print(d->position[0],3);     // 17 (m)
    Serial.print(" ");    Serial.print(d->position[1],3);     // 18 (m)
    Serial.print(" ");    Serial.print(d->heading,3);     // 19 (rad)
    Serial.print(" ");    Serial.print(d->velocity[0],3);     // 17 (m/s)
    Serial.print(" ");    Serial.print(d->velocity[1],3);     // 18 (m/s)
    Serial.print(" ");    Serial.print(d->turnrate,3);     // 19 (rad/s)
    Serial.println(" ");
    //delay(1);
    d++;
  }
  Serial.print("% send ");
  Serial.print(logsCnt);
  Serial.println(" log lines");
}

// ////////////////////////////////////////

/**
 * Global variables for timing */
bool isSampleTime = false;
float time_sec = 0;

/**
 * Global variables for control and pos */
float oldIntegral[3] = {0.0, 0.0, 0.0};
float posResults[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float turnAcc[2] = {0.0, 0.0};
float velAcc[2] = {0.0, 0.0};

// Timer that controls sample time (interrupt based)
IntervalTimer sampleTimer;

void timerInterrupt()
{
  isSampleTime = true;
  time_sec += sampleTimeUs / 1e6;
}

void setup()   // INITIALIZATION
{ // to be able to print to USB interface
  Serial.begin(12000000);
  sampleTimer.begin(timerInterrupt, sampleTimeUs);
  // initialize sensors, safety and display
  robot.setup(); // alive LED, display and battery
  motor.setup(); // motor voltage
  encoder.setup(); // motor encoders to velocity
  encoder.pulsPerRev = 48;
  imu2.setup(); // gyro and accelerometer
  motor.setPWMfrq(80000);
  usb.setup();
}

/**
 * Global variables for state machine */
int state = 0;               // actual sequence state
float endTime = 0;           // for current state
float desiredValue[2] = {0, 0};      // desired (reference) value send to controller

void start()
{ // Start timing
  // reset log and encoders
  logsCnt = 0;
  // change motor PWM frequency for sampling time test
  // should not be above 100000 (100kHz), default is 80kHz.
  motor.setPWMfrq(80000);
  time_sec = 0;
  encoder.encoder[0] = 0; // left motor encoder
  encoder.encoder[1] = 0; // right motor encoder
  // Update of display takes too long for fast sampling
  // so, disable during sequence.
  display.useDisplay = false;

  for(uint8_t i = 0; i < 3; i++) {
    oldIntegral[i] = 0;
  }
  turnAcc[0] = 0.0;
  velAcc[0] = 0.0;
  turnAcc[1] = 0.0;
  velAcc[1] = 0.0;
}

void stop(float after)
{ // stop, but allow continued logging for a while
  state = 999;
  desiredValue[0] = 0;
  desiredValue[1] = 0;
  // end log after an additional short time
  endTime = time_sec + after;
}

void finished()
{ // prepare for a new start
  // stop motors
  motor.motorVoltage[0] = 0; // left motor
  motor.motorVoltage[1] = 0;  // right motor
  // set default PWM frequency
  motor.setPWMfrq(80000);
  // start updating display
  display.useDisplay = true;
  // wait for next button press
  state = 0;
  printLog();
}

/**
 * Simple 2-step sequence */
void sequenceTwoSteps()
{ // this function is called at every sample time
  // and should never wait in a loop.
  // Update variables as needed and return.
  bool button;
  float pos = 0.0;
  float dir = 0.0;
  static float refPos[3] = {0.0, 0.0, 0.0};
  //
  // this is a state machine
  // state 0: wait for start button press
  // other states are part of a sequence

  /*
  Change in time
    endTime = time_sec + 0.020;
    break;
    ...
  if (time_sec > endTime)

  Change in position
  pos = sqrt(pow(posResults[4] - refPos[1], 2) + pow(posResults[5] - refPos[2], 2));
  if(pos >= 0.5)

  Change in direction
  dir = posResults[1] - refPos[0];
  if(dir <= -0.3)

  Save reference position
  refPos[0] = posResults[1];
  refPos[1] = posResults[4];
  refPos[2] = posResults[5];
  */

  switch (state)
  { // run mission, initial value
    case 0: // State 0 is just inactive, waiting for the start signal.
      button = digitalReadFast(PIN_START_BUTTON);
      if (button or robot.missionStart)
      { // starting a sequence
        // reset for new run
        start();
        // Prepare next state
        desiredValue[0] = 0; // reference value to the velocity controller
        desiredValue[1] = 0; // reference value to the turnrate controller
        refPos[0] = posResults[1];
        refPos[1] = posResults[4];
        refPos[2] = posResults[5];
        // to get start the log with no velocity
        endTime = time_sec + 0.020; // new state to end after 20ms
        state = 10;
      }
      break;
    case 10: // Waiting for first step.
      // test if this state is finished
      if (time_sec > endTime)
      { // change to next values
        desiredValue[0] = 0.75;
        desiredValue[1] = 0.0;
        refPos[0] = posResults[1];
        refPos[1] = posResults[4];
        refPos[2] = posResults[5];
        state = 11;
      }
      break;
    case 11:
      // test if this state is finished
      pos = sqrt(pow(posResults[4] - refPos[1], 2) + pow(posResults[5] - refPos[2], 2));
      if(pos >= 0.95)
      { // change to next values
        desiredValue[0] = 0.3;
        desiredValue[1] = -2.25;
        refPos[0] = posResults[1];
        refPos[1] = posResults[4];
        refPos[2] = posResults[5];
        state = 12;
      }
      break;
    case 12:
      // test if this state is finished
      dir = posResults[1] - refPos[0];
      if(dir <= -0.9)
      { // change to next values
        desiredValue[0] = 0.75;
        desiredValue[1] = 1.3;
        refPos[0] = posResults[1];
        refPos[1] = posResults[4];
        refPos[2] = posResults[5];
        state = 13;
      }
      break;
    case 13:
      // test if this state is finished
      dir = posResults[1] - refPos[0];
      if(dir * 0.5 >= 2.75)
      { // change to next values
        desiredValue[0] = 0.3;
        desiredValue[1] = -2.25;
        refPos[0] = posResults[1];
        refPos[1] = posResults[4];
        refPos[2] = posResults[5];
        state = 14;
      }
      break;
    case 14:
      // test if this state is finished
      dir = posResults[1] - refPos[0];
      if(dir <= -0.75)
      { // change to next values
        desiredValue[0] = 0.75;
        desiredValue[1] = 0.0;
        refPos[0] = posResults[1];
        refPos[1] = posResults[4];
        refPos[2] = posResults[5];
        state = 100;
      }
      break;
    case 100: // Maintain last step
      // stop after end time
      pos = sqrt(pow(posResults[4] - refPos[1], 2) + pow(posResults[5] - refPos[2], 2));
      if(pos >= 1.5)
      { // stop, but continue logging for a while (0.3 sec)
        stop(0.3); // actually sets state to 999 (i.e. default:)
      }
      break;
    default: // Hold until finished
      // this state is needed to enable a new start
      if (time_sec  > endTime)
        finished();
      break;
  }
}

/**
 * make the control */
void controlUpdate(float leftWheelVel, float rightWheelVel)
{ // do control during a mission only.
  //
  // Measured wheel velocity (positive is forward)
  // float velLeft = -encoder.motorVelocity[0];
  // float velRight = encoder.motorVelocity[1];
  //
  // Desired velocity
  // float velRef = desiredValue;
  //
  // For a start, the desiredValue
  // is just used as motor voltage
  //
  // Left motor voltage (-9V to +9V)
  motor.motorVoltage[0] = -leftWheelVel;
  // Right motor voltage (-9V to +9V)
  motor.motorVoltage[1] = rightWheelVel;
}

float integral(float x, float oldX, float oldY) {
  float y = (sampleTimeUs / 1e6) / 2.0 * (x + oldX) + oldY;
  return y;
}

void coordinateMotors() {
  float velLeft = -encoder.motorVelocity[0] * r2m;
  float velRight = encoder.motorVelocity[1] * r2m;
  float vel = (velLeft + velRight) / 2.0;

  float turn = (velRight - velLeft) / 0.15;
  float head = integral(turn, posResults[0], posResults[1]);
  float velX = vel * cos(head);
  float velY = vel * sin(head);
  
  float posX = integral(velX, posResults[2], posResults[4]);
  float posY = integral(velY, posResults[3], posResults[5]);

  posResults[0] = turn;
  posResults[1] = head;
  posResults[2] = velX;
  posResults[3] = velY;
  posResults[4] = posX;
  posResults[5] = posY;
}

void PIcontroller(float out[3], float KP, float tauI, float ref, float meas, float oldError, float oldIntegral, int saturate, int intSaturate) {
  float error = ref - meas;
  float intGain = KP / tauI;
  float integralVal = integral(intGain * error, intGain * oldError, oldIntegral);

  if(intSaturate > 0) {
    if(integralVal > intSaturate) {
      integralVal = intSaturate;
    } else if(integralVal < -intSaturate) {
      integralVal = -intSaturate;
    }
  }

  float propVal = KP * error;
  float u = propVal + integralVal;

  if(saturate > 0) {
    if(u > saturate) {
      u = saturate;
    } else if(u < -saturate) {
      u = -saturate;
    }
  }

  out[0] = u;
  out[1] = error;
  out[2] = integralVal;
}

void acc_limtier(float out[2], float KP, float ref, int saturate) {
  float uP = KP * (ref - out[0]);
  
  if(saturate > 0) {
    if(uP > saturate) {
      uP = saturate;
    } else if(uP < -saturate) {
      uP = -saturate;
    }
  }

  float u = integral(uP, out[1], out[0]);
  
  out[0] = u;
  out[1] = uP;
}

void updateLog()
{ // step response mode
  if (state > 0 and logsCnt < logsMax)
  {
    logs[logsCnt].time = time_sec;
    logs[logsCnt].state = state;
    logs[logsCnt].desiredValue[0] = desiredValue[0];
    logs[logsCnt].desiredValue[1] = desiredValue[1];
    logs[logsCnt].motorVolt[0] = -motor.motorVoltage[0]; // The left motor runs backwards, therefore the sign.
    logs[logsCnt].motorVolt[1] = motor.motorVoltage[1];
    logs[logsCnt].encoder[0] = -encoder.encoder[0];
    logs[logsCnt].encoder[1] = encoder.encoder[1];
    logs[logsCnt].motorVel[0] = -encoder.motorVelocity[0];
    logs[logsCnt].motorVel[1] = encoder.motorVelocity[1];
    logs[logsCnt].battery = robot.batteryVoltage;
    logs[logsCnt].gyro[0] = imu2.gyro[0];
    logs[logsCnt].gyro[1] = imu2.gyro[1];
    logs[logsCnt].gyro[2] = imu2.gyro[2];
    logs[logsCnt].position[0] = posResults[4];
    logs[logsCnt].position[1] = posResults[5];
    logs[logsCnt].heading = posResults[1];
    logs[logsCnt].velocity[0] = posResults[2];
    logs[logsCnt].velocity[1] = posResults[3];
    logs[logsCnt].turnrate = posResults[0];
    // the last part is offset of current measurement (in Amps)
    // take the initial current readings and add or subtract as needed
    // these values fits Tania: (+0.260A, -0.275A)
    logs[logsCnt].motorCurrent[0] = -(analogRead(A1) - 2024) * 0.0045 + 0.260;
    logs[logsCnt].motorCurrent[1] =  (analogRead(A0) - 2024) * 0.0045 - 0.275;
    logsCnt++;
  }
}

/**
* Main loop
* */
void loop ( void )
{ // init sample time
  float oldError[3] = {0.0, 0.0, 0.0};

  float turnCtrlArr[3] = {0.0, 0.0, 0.0};
  float leftCtrlArr[3] = {0.0, 0.0, 0.0};
  float rightCtrlArr[3] = {0.0, 0.0, 0.0};

  float leftRef = 0;
  float rightRef = 0;

  bool logReady = true;

  while ( true )
  { // main loop
    // loop until time for next sample
    if (isSampleTime) // start of new control cycle
    { // time for next tick
      isSampleTime = false;
      // read sensors
      imu2.tick();
      encoder.tick();

      // Position estimation
      coordinateMotors();

      // Calculate the PI values
      PIcontroller(turnCtrlArr, 0.5, 0.05, desiredValue[1], imu2.gyro[2] * 0.0175, oldError[2], oldIntegral[2], 0, 10000);

      acc_limtier(velAcc, 25.0, desiredValue[0], 3.0);
      acc_limtier(turnAcc, 25.0, turnCtrlArr[0] * 0.075, 3.0);

      leftRef = velAcc[0] - turnAcc[0];
      rightRef = velAcc[0] + turnAcc[0];

      PIcontroller(leftCtrlArr, 0.05, 0.1, leftRef / r2m, -encoder.motorVelocity[0], oldError[0], oldIntegral[0], 9, 10000);
      PIcontroller(rightCtrlArr, 0.05, 0.1, rightRef / r2m, encoder.motorVelocity[1], oldError[1], oldIntegral[1], 9, 10000);

      // Update old values from PI-controllers
      oldError[0] = leftCtrlArr[1];
      oldIntegral[0] = leftCtrlArr[2];
      oldError[1] = rightCtrlArr[1];
      oldIntegral[1] = rightCtrlArr[2];
      oldError[2] = turnCtrlArr[1];
      oldIntegral[2] = turnCtrlArr[2];

      // advance sequence (default is wait for start button)
      sequenceTwoSteps();
      //
      if (state > 0)
      { // Only if started
        // Calculate new motor voltage
        controlUpdate(leftCtrlArr[0], rightCtrlArr[0]);
      }
      // give value to actuators
      motor.tick();
      // save relevant values
      if(logReady == true) {
        updateLog();
        logReady = false;
      } else {
        logReady = true;
      }
      
      // support functions
      robot.tick();   // measure battery voltage etc.
      display.tick(); // update O-LED display
    }
    usb.tick(); // listen to incoming from USB
  }
}


/////////////////////////////////////////////////////////////////

