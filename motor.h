#ifndef MOTOR_H_
#define MOTOR_H_

struct motor {
  int   pwm;              //its an int so overflow problems don't happen
  char  pwm_pin;
  char  directiona;
  char  directionb;
  long  encoder_value;
  float command_velocity; //specified in rad/s
  float current_velocity; //specified in rad/s
};

void moveMotor(const motor &active_motor, const int &direction) {
  if (direction < 0) {
    digitalWrite(active_motor.directiona,LOW);
    digitalWrite(active_motor.directionb,HIGH);
  }
  else if (direction > 0) {
    digitalWrite(active_motor.directiona,HIGH);
    digitalWrite(active_motor.directionb,LOW);
  }
}

void setMotorSpeed(const motor &active_motor, const char &duty) {
  analogWrite(active_motor.pwm_pin, duty);
}

void stopMotor(const motor &active_motor) {
  digitalWrite(active_motor.directiona,LOW);
  digitalWrite(active_motor.directionb,LOW);
}

#endif
