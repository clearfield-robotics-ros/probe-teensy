
float loadValue = 0.0f;
float dloadValuedt = 0.0f;
long int lastTime = 0;

RunningAverage forceFilter(5); // samples
RunningAverage forceDerivativeFilter(5); // samples

void setupLoadCell() {
  // Determined with Matlab Script
  loadCell.set_scale(67590.7657); // 1/gain
  loadCell.set_offset(-0.8384);
  attachInterrupt(digitalPinToInterrupt(DAT), readForce, FALLING);
  forceFilter.clear();
  forceDerivativeFilter.clear();
}

bool newValue = false;
void readForce() { // ISR

  newValue = true; // For logging

  long int time = millis();
  double dt = (time - lastTime);

  if (dt > 10) { // reject false readings from DATA pin

    float incoming = loadCell.get_value();
    dloadValuedt = (incoming - loadValue) / (dt) * 1000;
    loadValue = incoming;

    forceFilter.addValue(loadValue);
    forceDerivativeFilter.addValue(dloadValuedt);

    lastTime = time;
  }
}

bool readyToRead() {
  if (newValue) {
    newValue = false;
    return true;
  }
  else return false;
}

float getRawForce() {
  return loadValue; // kg
}

float getForce() {
  return forceFilter.getAverage() - tare;
}

float getAbsForce() {
  float force = getForce();
  if (force < 0) force = 0; // Only Positive Values
  return force;
}

float getForceDerivative() {
//  return dloadValuedt; // kg/s
  return forceDerivativeFilter.getAverage();
}

float getNormalizedForceDerivative() {
  return getForceDerivative() / speedAdjustment; // kg/s
}


