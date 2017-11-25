// Code for Probe Linear Actuation System
// Author: David Robinson

#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <Encoder.h>
#include <HX711.h>

#define UPPER_SWITCH_PIN 20
#define LOWER_SWITCH_PIN 23

#define motorPWM 36
#define motorDIR 35
#define EncA 15
#define EncB 14
Encoder enc(EncA, EncB);

#define DAT   32
#define CLK   31
HX711 loadCell(DAT, CLK);
float tare = 0.0f;
int tareSamples = 0;

int state;    // state machine
static int initialState = 0;
#define ZERO  0
#define IDLE  1
#define PROBE 2
#define CALIB 3

bool debug = false;

void setup() {
  setupLoadCell();
  setupMotor();
  setupSwitches();
  setupROS();
  pinMode(13, OUTPUT);
  Serial.begin(9600);
}

int startupDelay = true;

void loop() {

  if (startupDelay) {
    delay(1500);
    digitalWrite(13, HIGH);
    setState(initialState);
    startupDelay = false;
  }

  if (debug) serialControl();
  readSwitches();
  stateMachine();
  runROS();
}

///////////////////////
// ZEROING FUNCTIONS //
///////////////////////

long unsigned int zeroTime = 0;
static unsigned int maxZeroTime = 500; //ms
bool recordTime = false;

void enterZero() {
  if (debug) Serial.println("Entered Zero State");
  tare = 0;
  tareSamples = 0;
  recordTime = false;
}

void zero() {
  if (!upperSwitch()) {
    runMotor(-100); // retract at 75%
  }
  else {
    runMotor(0);

    if (!recordTime) {
      zeroTime = millis();
      recordTime = true;
    }

    tare += getRawForce();
    ++tareSamples;

    if ( (millis() - zeroTime) > maxZeroTime) {
      tare = tare / tareSamples;
      if (debug) {
        Serial.print("Limit Switch Found at ");
        Serial.print(getMotorPosition());
        Serial.print(" revs, Tare at ");
        Serial.print(tare);
        Serial.print(" kg(s)");
        Serial.println();
      }
      setMotorZero();
      setState(IDLE);
    }
  }
}

////////////////////
// IDLE FUNCTIONS //
////////////////////

void enterIdle() {
  if (debug) Serial.println("Entered Idle State");
}

void idle()
{
  runMotor(0);
}

///////////////////////////
// CALIBRATION FUNCTIONS //
///////////////////////////

static float forceLimit = 10.0f; // safety
float maxCalibForce = 0.0f;      // hardcoded for speed of demo
bool calibrated = false;          // hardcoded for speed of demo

void enterCalibration() {
  maxCalibForce = 0.0f;
  calibrated = false;
  if (debug) Serial.println("Entered Calibration State");
}

void calibration() {

  runMotor(50);

  if (getForce() > maxCalibForce)
    maxCalibForce = getForce();

  if (debug) {
    Serial.print("Max Force: ");
    Serial.println(maxCalibForce);
  }
  // Exit Conditions
  if (lowerSwitch()) {
    if (debug) {
      Serial.print("Calibrated with Maximum Force of ");
      Serial.println(maxCalibForce);
    }
    calibrated = true;
    setState(ZERO);
  }
  else if (getRawForce() > forceLimit) {
    if (debug)
      Serial.println("EVENT: Exited because safe load cell force exceeded, Try Again!");
    setState(ZERO);
  }
}

/////////////////////
// PROBE FUNCTIONS //
/////////////////////

static float finalPWM = 0.3f;
static float speedReductionFactor = 0.0f;
float speedAdjustment;

static float classificationThreshold = 20.0f;
static float maxCalibForceMultiplier = 1.5f;

bool object = false;

void enterProbe() {

  if (!calibrated) {
    if (debug) Serial.println("System not Calibrated...");
    setState(IDLE);
  }
  else {
    if (debug) Serial.println("Entering Probe State");
    speedReductionFactor = (1 - finalPWM) / (finalPWM * maxCalibForce);
    object = false;
  }
}

void probe()
{
  speedAdjustment = 1 / (getAbsForce() * speedReductionFactor + 1);
  runMotor(100 * speedAdjustment);


  // Exit Conditions
  if (getNormalizedForceDerivative() > classificationThreshold) exitProbe(0);

  else if (getAbsForce() > (maxCalibForce * maxCalibForceMultiplier)) exitProbe(1);

  else if (lowerSwitch()) exitProbe(2);

  else if (getRawForce() > forceLimit) exitProbe(3);

  if (debug && readyToRead()) logValues();
}

void exitProbe(int exitCode) {

  setState(ZERO);

  switch (exitCode)
  {
    case 0:
      if (debug) Serial.println("Force Derivative Limit Exceeded");
      object = true;
      break;
    case 1:
      if (debug) Serial.println("Max Calibration Force Exceeded");
      object = true;
      break;
    case 2:
      if (debug) Serial.println("End of Linear Travel");
      break;
    case 3:
      if (debug) Serial.println("Safe load cell force exceeded");
    default:
      break;
  }

  ROSContactMsg();
}

bool objectFound() {
  return object;
}

void logValues() {
  Serial.print(millis());
  Serial.print("\t");
  Serial.print(getMotorPosition());
  Serial.print("\t");
  Serial.print(speedAdjustment);
  Serial.print("\t");
  Serial.print(getForce());
  Serial.print("\t");
  Serial.println(getNormalizedForceDerivative());
}

