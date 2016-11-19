
#define MIN_SAMPLE_INTERVAL 100 //0.1 ms

typedef struct
{
  //Tuning parameters
  //The monotonic system time at the last pid step
  uint64_t last_time;
  //The desired value of the measured quantity (input)
  double setpoint;
  //P constant
  double kp;
  //I constant
  double ki;
  //D constant
  double kd;
  //How frequently to step
  uint64_t step_interval_us;
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
  bool enabled;
  //The output
  double output;
} pid_t;

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

double pid_step(pid_t *p, double input)
{
  //Do not change any values if the PID is disabled
  if (!p->enabled) {
    return;
  }
  uint64_t now = now_in_us();
  int32_t time_since_last_step = (now - p->last_time);
  //Do not step if insufficient time has passed
  if (time_since_last_step < p->sample_interval_us) {
    return;
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
  if (output > p->output_maximium_clamp) {
    output = p->output_maximum_clamp;
  }
  if (output < p->output_minimum_clamp) {
    output = p->output_minimum_clamp;
  }

  //Store state needed for next time
  p->last_input = input;
  p->last_time = now;
}

void pid_set_tuning(pid_t *p, double kp, double ki, double kd)
{
  double sample_interval_sec = ((double)p->sample_interval_us) / 1000000;
  p->kp = kp;
  p->ki = ki*sample_interval_sec;
  p->kd = kd*sample_interval_sec;
}

void pid_set_sample_interval(pid_t *p, uint64_t new_sample_interval_us)
{
  if (new_sample_interval_us > MIN_SAMPLE_INTERVAL)
  {
    double ratio = (double) new_sample_interval_us /
                   (double) p->sample_interval_us;
    p->ki *= ratio;
    p->kd /= ratio;
    p->sample_interval_us = new_sample_interval_us;
  }
}

void pid_set_output_clamp(pid_t *p, double min, double max)
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

void pid_set_enabled(pid_t *p, bool enabled, double input)
{
  if (enabled && !p->enabled) {
    pid_initialize(p, input);
  }
  p->enabled = enabled;
}
//Used to make sure that upon enabling the PID controller, there
//is not a transient
static void pid_initialize(pid_t *p, double input)
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
