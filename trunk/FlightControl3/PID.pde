// PID

// Modified from http://www.arduino.cc/playground/Main/BarebonesPIDForEspresso
int updatePID(int targetPosition, int currentPosition, struct PID *p) {
  int error = targetPosition - currentPosition;  

  p->integratedError += error;

  // Limit the integrated error by the windupGuard
  if (p->integratedError < -windupGuard) {
    p->integratedError = -windupGuard;
  } else if (p->integratedError > windupGuard) {
    p->integratedError = windupGuard;
  }
  
  int dTerm = (p->D * (currentPosition - p->lastPosition)) / 10;
  p->lastPosition = currentPosition;
  return ((p->P * error) / 10) + ((p->I * p->integratedError) / 10) + dTerm;
}

void zeroIntegralError() {
  for(int n=0;n<3;n++) {
    pid[n].integratedError = 0;
  }
}

