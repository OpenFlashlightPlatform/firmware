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

static double scramTrip = 45.0;

static pid_t vPID;
static pid_t iPID;

#define SHUNT_M_FACTOR 0.230210715726
#define SHUNT_C_FACTOR 282.89660446
double getShuntMilliAmps() {
  int32_t val = (int16_t) skookumADC_sampleDifferential(SHUNT_NEG, SHUNT_POS, SKOOKUM_GAIN_8X);
  return ((double)val) * (SHUNT_M_FACTOR) + (SHUNT_C_FACTOR);
}

#define LOAD_M_FACTOR 0.00250697726152
#define LOAD_C_FACTOR 0.425123865184
double getLoadVoltage() {
  int32_t val = (int16_t) skookumADC_sampleDifferential(SKOOKUM_ADC_GROUND, VOUT, SKOOKUM_GAIN_1X);
  return ((double)val) * (LOAD_M_FACTOR) + (LOAD_C_FACTOR);
}

void engineSetSCRAM(double volts) {
  scramTrip = volts;
}
void engineCycle() {
  double loadv = getLoadVoltage();

}
void initEngine() {
  skookumADC_init(SHUNT_NEG);
  skookumADC_init(SHUNT_POS);
  skookumADC_init(VOUT);
}
