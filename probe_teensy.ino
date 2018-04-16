  // Code for Probe Linear Actuation System
// Author: David Robinson

#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <probe/probe_data.h>

#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <Encoder.h>
#include <RunningAverage.h>
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
bool contact_flag = false;

void setup() {
  setupLoadCell();
  setupMotor();
  setupSwitches();
  if (!debug) setupROS();
  setState(initialState);
  pinMode(13, OUTPUT);
  Serial.begin(9600); 
}

void loop() {
  digitalWrite(13, HIGH);
  if (debug) serialControl();
  readSwitches();
  stateMachine();
  updateMotorSpeed();
  if (!debug) runROS();
}

///////////////////////
// ZEROING FUNCTIONS //
///////////////////////

bool firstZero = true;
long unsigned int zeroTime = 0;
static unsigned int alottedZeroTime = 500; //ms
bool recordTime = false;

void enterZero() {
  if (debug) Serial.println("Entered Zero State");
  tare = 0;
  tareSamples = 0;
  recordTime = false;
}

void zero() {
  if (!upperSwitch() && firstZero) {
    runMotor(-30); // retract slowly for first zero
  }
  else if (!upperSwitch() && !firstZero) {
    if (getMotorPosition() > 50) runMotor(-100); // run fast till close to zero
    else runMotor(-(getMotorPosition() + 50));
  }
  else {
    runMotor(0);
    firstZero = false;

    if (!recordTime) {
      zeroTime = millis();
      recordTime = true;
    }

    tare += getRawForce();
    ++tareSamples;

    if ( (millis() - zeroTime) > alottedZeroTime) {
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

static float forceLimit = 8.0f;   // safety
float maxCalibForce = 8.0f;       // hardcoded for speed of demo
bool calibrated = true;           // hardcoded for speed of demo

void enterCalibration() {
  maxCalibForce = 0.0f;
  calibrated = false;
  if (debug) Serial.println("Entered Calibration State");
}

void calibration() {

  runMotor(50);

  if (getForce() > maxCalibForce)
    maxCalibForce = getForce();

  if (debug && readyToRead()) {
    Serial.print("Max Force: ");
    Serial.println(maxCalibForce);
  }
  // Exit Conditions
  if (lowerSwitch()) {
    if (debug) {
      Serial.print("Calibrated with Maximum Force of ");
      Serial.println(maxCalibForce);
    }
    
    runMotor(0);
    delay(500);
    
    calibrated = true;
    setState(ZERO);
  }
  else if (getRawForce() > forceLimit) {
    if (debug)
      Serial.println("Exited because safe load cell force exceeded");
    
    runMotor(0);
    delay(500);
    
    calibrated = true;
    setState(ZERO);
  }
}

/////////////////////
// PROBE FUNCTIONS //
/////////////////////

static float finalPWM = 0.3f;
static float speedReductionFactor = 0.0f;
float speedAdjustment;

static float classificationThreshold = 40.0f;
static float calibForceFactor = 1.5f;

bool object = false;
bool probeConfirm = false;

void enterProbe() {

  if (!calibrated) {
    if (debug) Serial.println("System not Calibrated...");
    setState(IDLE);
  }
  else {
    if (debug) Serial.println("Entering Probe State");
    speedReductionFactor = (1 - finalPWM) / (finalPWM * maxCalibForce);
    object = false;
    probeConfirm = false;
  }
}

void performProbe()
{

  speedAdjustment = 1 / (getAbsForce() * speedReductionFactor + 1);
  runMotor(100 * speedAdjustment);

  if (getContactForce() > classificationThreshold
      && getAbsForce() > maxCalibForce * 0.05) {
    if (!probeConfirm) {
      probeConfirm = true;
      if (debug) Serial.println("Probe Confirming...");
      runMotor(0);
//      delay(100);
    }
    else exitProbe(0);
  }

  // secondary exit conditions
  if (getAbsForce() > (maxCalibForce * calibForceFactor))   exitProbe(1);
  else if (lowerSwitch())                                   exitProbe(2);
  else if (getRawForce() > forceLimit)                      exitProbe(3);

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
//      object = true;
      break;
    case 2:
      if (debug) Serial.println("End of Linear Travel");
      break;
    case 3:
      if (debug) Serial.println("Safe load cell force exceeded");
    default:
      break;
  }

//  if (!debug) ROSContactMsg();
  if (!debug) contact_flag = true;
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
  Serial.print(getMotorSpeed());
  Serial.print("\t");
  Serial.print(getForce());
  Serial.print("\t");
  Serial.print(getForceDerivative());
  Serial.print("\t");
  Serial.print(getNormalizedForceDerivative());
  Serial.print("\t");
  Serial.println(getContactForce());
}

