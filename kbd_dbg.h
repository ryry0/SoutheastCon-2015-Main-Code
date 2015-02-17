#ifndef KBD_DBG_H_
#define KBD_DBG_H_

void readKeyboard(movement_vector_t &movement_vector, states_t &state) {
  char incoming_byte = 0;

  //read the serial
  if (Serial.available() > 0) {
    incoming_byte = Serial.read();

    switch(incoming_byte) {
      case ' ': //stop line following
        movement_vector.x_velocity = 0;
        movement_vector.y_velocity = 0;
        movement_vector.angular_velocity = 0;
        for (int i = 0; i < NUM_MOTORS; ++i) {
          motors[i].pwm = 0;
          motors[i].command_position = 0;
          motors[i].command_velocity = 0;
          motors[i].encoder_value = 0;
          motor_encoders[i] = 0;
          motor_pid_data[i].pid_output = 0;
          motor_pid_data[i].previous_error = 0;
          motor_pid_data[i].integral_error = 0;
          stopMotor(motors[i]);
        }
        state = STOPPED;
        break;

      case 'F': //start
        state = FOLLOW_LINE;
        LINE_SERIAL.write(LINE_SERIAL_RESET);
        digitalWrite(LED_PIN, HIGH);
        delay(2000);
        digitalWrite(LED_PIN, LOW);
        LINE_SERIAL.write(LINE_SERIAL_START);
        break;

      case 'D':
        state = DBG_LINE_SENSORS;
        LINE_SERIAL.write(LINE_SERIAL_START);
        break;

      case 'R':
        LINE_SERIAL.write(LINE_SERIAL_RESET);
        break;

      case 'S':
        LINE_SERIAL.write(LINE_SERIAL_START);
        break;

      case 'w':
        movement_vector.x_velocity += 0.1;
        break;

      case 's':
        movement_vector.x_velocity -= 0.1;
        break;

      case 'a':
        movement_vector.y_velocity += 0.1;
        break;

      case 'd':
        movement_vector.y_velocity -= 0.1;
        break;

      case 'e':
        movement_vector.angular_velocity -= .24;//.3;
        break;

      case 'q':
        movement_vector.angular_velocity += .24;//.3;
        break;

      case 'l':
        digitalWrite(LED_PIN, LOW);
        break;

      case 'L':
        digitalWrite(LED_PIN, HIGH);
        break;

      default:
        break;
    } //end switch
  } //end if
} //end readKeyboard();
#endif
