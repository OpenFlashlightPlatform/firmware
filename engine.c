/* Herein lies the engine. She may not be as loud as a mechanical
 * engine but she's no less intricate.
 *
 * If you are gonna stick ur pokey bits in here, make sure you
 * have the circuit powered from a current limited supply and
 * ye have yer mother on speed dial
 */

/* These were determined by linear regression of the measured ADC counts
 * of a calibrated current source.
 * @see bluesun.institute/2016/12/calibrating-shunt.html
 */

#include "Arduino.h"
#include "bluesun.h"

static double scramTrip = 45.0;

pid_controller_t vPID;
static pid_controller_t iPID;

#define VPID_T 0.005
//This is a reasonable KP, with the bias of 0.65 and an input of 12V
// it is correct for +- 30-40V
// 0.65 - 0.01*5 = 0.60 = 30V steady state
// 0.65 + 0.01*5 = 0.70 = 40V steady state
#define VPID_KP 0.005
#define VPID_KI 0.1 * VPID_T
#define VPID_KD 0.00 * VPID_T
//The theoretical duty cycle to obtain ~35V from 12V
#define VPID_OBIAS 0.65

#define BOOST_TOP 128

#define SHUNT_M_FACTOR 0.230210715726
#define SHUNT_C_FACTOR 282.89660446
// double getShuntMilliAmps() {
//   int32_t val = (int16_t) skookumADC_sampleDifferential(SHUNT_NEG, SHUNT_POS, SKOOKUM_GAIN_8X);
//   return ((double)val) * (SHUNT_M_FACTOR) + (SHUNT_C_FACTOR);
// }

#define LOAD_M_FACTOR 0.00250697726152
#define LOAD_C_FACTOR 0.425123865184
// double getLoadVoltage() {
//   int32_t val = (int16_t) skookumADC_sampleDifferential(SKOOKUM_ADC_GROUND, VOUT, SKOOKUM_GAIN_1X);
//   return ((double)val) * (LOAD_M_FACTOR) + (LOAD_C_FACTOR);
// }

void engineSetSCRAM(double volts) {
  scramTrip = volts;
}

void SCRAM() {
  //disable interrupts and spin
  __disable_irq();
  //Switch load on
   skookumPWM_update(LOAD_SW, 100, 100);
   skookumPWM_update(BOOST_SW, 0, 100);
  while(1) {
    digitalWrite(GP0, 1);
    digitalWrite(GP0, 0);
    digitalWrite(GP0, 1);
  }
}
static void processVPID()
{
  int16_t val = (int16_t) skookumADC_getResult();
  double voltage = ((double)val) * (LOAD_M_FACTOR) + (LOAD_C_FACTOR);
  if (voltage > scramTrip) {
    SCRAM();
  }
  double duty = pid_step(&vPID, voltage) + VPID_OBIAS;
  duty *= BOOST_TOP;
  duty = round(duty);
  skookumPWM_update(BOOST_SW, (int)duty, BOOST_TOP);
}

static void processIPID()
{
  int16_t val = (int16_t) skookumADC_getResult();
  double current_mA = ((double)val) * (LOAD_M_FACTOR) + (LOAD_C_FACTOR);
}

static void processLP()
{

}

static inline void sampleI()
{
  skookumADC_beginSampleDifferential(SHUNT_NEG, SHUNT_POS, SKOOKUM_GAIN_8X);
}

static inline void sampleV()
{
  skookumADC_beginSampleDifferential(SKOOKUM_ADC_GROUND, VOUT, SKOOKUM_GAIN_1X);
}

static void sampleLP()
{

}
//It is important for control stability that there is no jitter between
//actions in the PID loops. So we have a four stroke engine:
//stroke 1: process V PID and start sample of I
//stroke 2: process I PID and start sample of V
//stroke 3: process V PID and start sample of LP
//stroke 4: copy LP sample to appropriate location and start sample of V
// LP cycles between
// VBAT, GP0, GP1
//This gives us a V PID frequency of 200 Hz (on 5ms ticks)
// an I PID frequency of 50 Hz
// and a VBAT, GP0 and GP1 frequency of ~16 Hz
static uint8_t engineStroke;
static void engineCycle() {
  //double loadv = getLoadVoltage();
  digitalWrite(GP0, 1);
  switch (engineStroke) {
    case 0:
      processVPID();
      sampleI();
      break;
    case 1:
      processIPID();
      sampleV();
      break;
    case 2:
      processVPID();
      sampleLP();
      break;
    case 3:
      processLP();
      sampleV();
      break;
  }
  engineStroke++;
  engineStroke &= 0b11;
  digitalWrite(GP0, 0);
}
static uint32_t holdoffCount;
void TC4_Handler (void)
{
  //This is the main engine handler
  if (holdoffCount > 0) {
    holdoffCount --;
    if (holdoffCount == 0) {
      engineStroke = 0;
      sampleV();
    }
  } else {
    engineCycle();
  }


  //Clear the flag
  TC4->COUNT16.INTFLAG.bit.MC0 = 1;
}

//The engine TC runs at 16384 Hz. A tick every 5ms is then 164 ticks
#define ENGINE_TICKS 163
void initEngine() {
  skookumADC_init(SHUNT_NEG);
  skookumADC_init(SHUNT_POS);
  skookumADC_init(VOUT);
  skookumADC_init(VBATT);
  skookumPWM_init(BOOST_SW);
  pinMode(LOAD_SW, OUTPUT);
  pinMode(GP0, OUTPUT);
  digitalWrite(GP0, 0);
  digitalWrite(LOAD_SW, 1);
  //It's really important that the engine keeps running without
  //any interruptions. So we run it an interupt priority 0, with the
  //ADC completion handlers happening interrupt priority 1 (less important).
  //All other Arduino nvic priorities have been changed to 2 and 3. The
  //control logic will occur at normal thread priority and on a timer
  //at interrupt priority 2.

  NVIC_DisableIRQ(TC4_IRQn);
  NVIC_ClearPendingIRQ(TC4_IRQn);
  NVIC_SetPriority(TC4_IRQn, 0);

  //We use 32768khz gclk
  GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK1 | GCLK_CLKCTRL_ID(GCM_TC4_TC5));
  while (GCLK->STATUS.bit.SYNCBUSY == 1);

  //TC4 is used by us for engine tasks
  //TC5 is used for lpri tasks
  TC4->COUNT16.CTRLA.bit.SWRST = 1;
  while (TC4->COUNT16.STATUS.bit.SYNCBUSY);
  TC4->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16 | TC_CTRLA_WAVEGEN_MFRQ | TC_CTRLA_PRESCALER_DIV2;
  while (TC4->COUNT16.STATUS.bit.SYNCBUSY);
  TC4->COUNT16.CC[0].reg = ENGINE_TICKS;
  while (TC4->COUNT16.STATUS.bit.SYNCBUSY);
  TC4->COUNT16.INTENCLR.reg = 0xFF;
  while (TC4->COUNT16.STATUS.bit.SYNCBUSY);
  TC4->COUNT16.INTENSET.bit.MC0 = 1;
  while (TC4->COUNT16.STATUS.bit.SYNCBUSY);

  //Start the first sample

  //Configure the PID controllers
  //The engine will start in "TRACKING_DISENGAGED" mode
  holdoffCount = 100;
  vPID.kp = VPID_KP;
  vPID.ki = VPID_KI;
  vPID.kd = VPID_KD;
  vPID.setpoint = 35;
  vPID.output_maximum_clamp = 0.80 - VPID_OBIAS;
  vPID.output_minimum_clamp = 0.0 - VPID_OBIAS;
  vPID.enabled = true;
  vPID.output = VPID_OBIAS;
  vPID.iterm = -VPID_OBIAS;
  vPID.last_input = 12;

  NVIC_EnableIRQ(TC4_IRQn);
  TC4->COUNT16.CTRLA.bit.ENABLE = 1;


}
