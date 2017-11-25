
void stateMachine() {
  switch (state)
  {
    case ZERO: zero();
      break;
    case IDLE: idle();
      break;
    case PROBE: probe();
      break;
    case CALIB: calibration();
    default:
      break;
  }
}

void setState(int s) {
  switch (s) // State Machine
  {
    case ZERO:
      state = ZERO;
      enterZero();
      break;
    case IDLE:
      state = IDLE;
      enterIdle();
      break;
    case PROBE:
      state = PROBE;
      enterProbe();
      break;
    case CALIB:
      state = CALIB;
      enterCalibration();
    default:
      break;
  }
}
