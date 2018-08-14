#include "bluesun.h"

extern pid_controller_t vPID;

void setup() {
   skookumPWM_init(LOAD_SW);
   skookumPWM_update(LOAD_SW, 100, 100);
   initEngine();
   vPID.setpoint = 36.0;
}


void loop() {

 // vPID.setpoint = 36.0;

  
}
