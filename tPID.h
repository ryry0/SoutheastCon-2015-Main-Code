#ifndef PID_H_
#define PID_H_

#include <Arduino.h>
//Use this data structure to create a customized PID per motor
template <typename T>
struct pid_data {
  T proportional_gain;
  T integral_gain;
  T derivative_gain;
  T previous_error;
  T integral_error;
  T integral_guard;
  T pid_output;
};

template <typename T>
void setPIDConstants(pid_data &pid, T proportional_gain,
                     T integral_gain, T derivative_gain) {
  pid.proportional_gain = proportional_gain;
  pid.integral_gain = integral_gain;
  pid.derivative_gain = derivative_gain;
  pid.integral_guard = integral_guard;
  pid.previous_error = 0;
  pid.integral_error = 0;
}

template <typename T>
void updatePID(pid_data &pid, T current_error, T delta_t) {
  T error_differential = 0;

  pid.integral_error += current_error * delta_t;
  pid.integral_error = constrain(pid.integral_error, -pid.integral_guard,
      pid.integral_guard);

  error_differential = (current_error - pid.previous_error)/delta_t;

  pid.pid_output =  (pid.proportional_gain * current_error) +
    (pid.integral_gain    * pid.integral_error) +
    (pid.derivative_gain  * error_differential);

  pid.previous_error = current_error;
} //end pidControl()

//fixed update PID is meant to be called at constant time intervals,
//therefore it does not need delta_t
template <typename T>
void fixedUpdatePID(pid_data &pid, T current_error) {
  T error_differential = 0;

  pid.integral_error += current_error;
  pid.integral_error = constrain(pid.integral_error, -pid.integral_guard,
      pid.integral_guard);

  error_differential = (current_error - pid.previous_error);

  pid.pid_output =  (pid.proportional_gain * current_error) +
                    (pid.integral_gain    * pid.integral_error) +
                    (pid.derivative_gain  * error_differential);

  pid.previous_error = current_error;
} //end pidControl()
#endif
