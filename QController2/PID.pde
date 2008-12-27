
#define P_GAIN 1.0;


float updatePID(int current, int target) { 
  float pTerm = (target - current) * P_GAIN;
  
  return pTerm;
}
