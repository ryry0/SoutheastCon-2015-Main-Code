#ifndef MOTOR_H_
#define MOTOR_H_

enum directions_t {DIRECTION_1, DIRECTION_2};

struct motor {
  int   pwm;              //its an int so overflow problems don't happen
  char  pwm_pin;
  char  directiona;
  char  directionb;
  long  encoder_value;
  float command_velocity; //specified in rad/s
  float current_velocity; //specified in rad/s
  float command_position; //specified in encoder ticks
};

void moveMotor(const motor &active_motor) {
  if (active_motor.command_velocity < 0) {
    digitalWrite(active_motor.directiona,LOW);
    digitalWrite(active_motor.directionb,HIGH);
  }
  else if (active_motor.command_velocity > 0) {
    digitalWrite(active_motor.directiona,HIGH);
    digitalWrite(active_motor.directionb,LOW);
  }
}

void setMotorSpeed(const motor &active_motor, const char &duty) {
  analogWrite(active_motor.pwm_pin, duty);
}

void setMotorDirection(const motor &active_motor,
    const directions_t &direction) {
   if (direction == DIRECTION_1) {
    digitalWrite(active_motor.directiona,LOW);
    digitalWrite(active_motor.directionb,HIGH);
  }
  else if (direction == DIRECTION_2) {
    digitalWrite(active_motor.directiona,HIGH);
    digitalWrite(active_motor.directionb,LOW);
  }
}


void stopMotor(const motor &active_motor) {
  digitalWrite(active_motor.directiona,LOW);
  digitalWrite(active_motor.directionb,LOW);
}

#endif
