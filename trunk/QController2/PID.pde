
#define P_GAIN 1.5;


float updatePID(int current, int target) { 
  float pTerm = (target - current) * P_GAIN;
  
  return pTerm;
}
