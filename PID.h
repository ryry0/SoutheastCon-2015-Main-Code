#ifndef PID_H_
#define PID_H_

//Use this data structure to create a customized PID per motor
struct pid_data {
  float proportional_gain;
  float integral_gain;
  float derivative_gain;
  float previous_error;
  float integral_error;
  float pid_output;
};

void setPIDConstants(pid_data &pid, float proportional_gain,
                     float integral_gain, float derivative_gain) {
  pid.proportional_gain = proportional_gain;
  pid.integral_gain = integral_gain;
  pid.derivative_gain = derivative_gain;
  pid.previous_error = 0;
  pid.integral_error = 0;
}

void updatePID(pid_data &pid, float current_error, float delta_t) {
  float error_differential = 0;

  pid.integral_error += current_error * delta_t;
  error_differential = (current_error - pid.previous_error)/delta_t;

  pid.pid_output =  (pid.proportional_gain * current_error) +
                    (pid.integral_gain    * pid.integral_error) +
                    (pid.derivative_gain  * error_differential);

  pid.previous_error = current_error;
} //end pidControl()

//fixed update PID is meant to be called at constant time intervals,
//therefore it does not need delta_t
void fixedUpdatePID(pid_data &pid, float current_error) {
  float error_differential = 0;

  pid.integral_error += current_error;
  error_differential = (current_error - pid.previous_error);

  pid.pid_output =  (pid.proportional_gain * current_error) +
                    (pid.integral_gain    * pid.integral_error) +
                    (pid.derivative_gain  * error_differential);

  pid.previous_error = current_error;
} //end pidControl()
#endif
