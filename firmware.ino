#include "bluesun.h"

void setup() {
  initEngine();
  skookumPWM_init(BOOST_SW);
}

void loop() {
  char buf [100];
  //pinMode(BOOST_SW, OUTPUT);
  for (int i = 0; i < 100; i++) {
    //digitalWrite(BOOST_SW, 1);
    //digitalWrite(BOOST_SW, 0);
    skookumPWM_update(BOOST_SW, i, 100);
    delay(100);
  }
  
  // uint32_t then = micros();
  // double shunt_ma = getShuntMilliAmps();
  // uint32_t now = micros();
  // sprintf(buf, "shunt reads %d mA, sample took %d us\n", (int)shunt_ma, now-then);
//  int32_t count = getVoutCount();
//  sprintf(buf, "%d \n",(int)count);
//  SerialUSB.write(buf);
  
}
