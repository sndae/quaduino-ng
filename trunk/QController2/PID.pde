
float P_GAIN[] = { 1.0, 1.0, 0.2 };
float I_GAIN[] = { 0.5, 0.5, 0.0 };
float D_GAIN[] = { 2.5, 2.5, 0.0 };

float integral[] = { 0, 0, 0 };
float last[] = { 0, 0, 0 };

float updatePID(int index, int current, int target) { 
  float error = target - current;
  integral[index] = constrain(integral[index]+error, -100, 100);
  float derivate = current - last[index];
  last[index] = current;  
//  return error*P_GAIN + integral[index]*I_GAIN - derivate*D_GAIN;
  return error*P_GAIN[index] + integral[index]*I_GAIN[index] - derivate*D_GAIN[index];
}
