

#include "bluesun.h"

#include <stdint.h>

#define MIN_SAMPLE_INTERVAL 100 //0.1 ms


/*working variables*/
// unsigned long lastTime;
// double Input, Output, Setpoint;
// double ITerm, lastInput;
// double kp, ki, kd;
// int SampleTime = 1000; //1 sec
// double outMin, outMax;
// bool inAuto = false;
//
// #define MANUAL 0
// #define AUTOMATIC 1

double pid_step(pid_controller_t *p, double input)
{
  //Do not change any values if the PID is disabled
  if (!p->enabled) {
    return -1;
  }

  //Calculate the difference between the intended observed value
  //and the actual observed value
  double error = p->setpoint - input;
  //Add the error to our integral term, with its present weighting
  //factor
  p->iterm += (p->ki * error);
  //Clamp the integral term, to prevent "windup". If this is not done,
  //even though the output is clamped, the iterm will take long to
  //respond to future changes as it will keep moving outside the
  //clamp zone.
  if (p->iterm > p->output_maximum_clamp) {
    p->iterm = p->output_maximum_clamp;
  }
  if (p->iterm < p->output_minimum_clamp) {
    p->iterm = p->output_minimum_clamp;
  }
  //Calculate the the difference in the observed quantity. This is
  //similar to the difference in the error, except it does not
  //exhibit a large jump when the setpoint is changed.
  double input_delta = input - p->last_input;

  //Compute the output
  //The iterm already contains ki.
  //and the input delta is negative because the difference
  //in error is equal to the negative of the difference in
  //the input
  p->output = ((p->kp * error) +
               (p->iterm) +
               (p->kd * -input_delta));

  //Clamp the output
  if (p->output > p->output_maximum_clamp) {
    p->output = p->output_maximum_clamp;
  }
  if (p->output < p->output_minimum_clamp) {
    p->output = p->output_minimum_clamp;
  }

  //Store state needed for next time
  p->last_input = input;

  return p->output;
}

void pid_set_tuning(pid_controller_t *p, double kp, double ki, double kd)
{
  p->kp = kp;
  p->ki = ki;
  p->kd = kd;
}

// void pid_set_sample_interval(pid_t *p, uint64_t new_sample_interval_us)
// {
//   if (new_sample_interval_us > MIN_SAMPLE_INTERVAL)
//   {
//     double ratio = (double) new_sample_interval_us /
//                    (double) p->sample_interval_us;
//     p->ki *= ratio;
//     p->kd /= ratio;
//     p->sample_interval_us = new_sample_interval_us;
//   }
// }

void pid_set_output_clamp(pid_controller_t *p, double min, double max)
{
  if(min < max) {
    return;
  }
  p->output_maximum_clamp = max;
  p->output_minimum_clamp = min;

  //Clamp the existing output
  if (p->output > max) {
    p->output = max;
  }
  if (p->output < min) {
    p->output = min;
  }

  //Also clamp the integral term
  if (p->iterm > max) {
    p->iterm = max;
  }
  if (p->iterm < min) {
    p->iterm = min;
  }
}

//Used to make sure that upon enabling the PID controller, there
//is not a transient
static void pid_initialize(pid_controller_t *p, double input)
{
  p->last_input = input;
  p->iterm = p->output;
  //Clamp the iterm in case the output exceeds
  if (p->iterm > p->output_maximum_clamp) {
    p->iterm = p->output_maximum_clamp;
  }
  if (p->iterm < p->output_minimum_clamp) {
    p->iterm = p->output_minimum_clamp;
  }
}

void pid_set_enabled(pid_controller_t *p, uint8_t enabled, double input)
{
  if (enabled && !p->enabled) {
    pid_initialize(p, input);
  }
  p->enabled = enabled;
}

