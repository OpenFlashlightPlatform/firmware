#ifndef __BLUESUN_H__
#define __BLUESUN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>


typedef struct
{
  //Tuning parameters

  //The desired value of the measured quantity (input)
  double setpoint;
  //P constant
  double kp;
  //I constant
  double ki;
  //D constant
  double kd;
  //The maximum value permitted on the output
  double output_maximum_clamp;
  //The minimum value permitted on the output
  double output_minimum_clamp;

  //PID state
  //Integrative term
  double iterm;
  //The value of the input on the last step
  double last_input;
  //If the PID is disabled
  uint8_t enabled;
  //The output
  double output;
} pid_controller_t;

double pid_step(pid_controller_t *p, double input);
void pid_set_tuning(pid_controller_t *p, double kp, double ki, double kd);
void pid_set_output_clamp(pid_controller_t *p, double min, double max);
void pid_set_enabled(pid_controller_t *p, uint8_t enabled, double input);

//Sample the current through the load
double getShuntMilliAmps();

//Get the engine all set up
void initEngine();

#ifdef __cplusplus
}
#endif

#endif
