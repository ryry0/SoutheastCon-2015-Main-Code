#ifndef KBD_DBG_H_
#define KBD_DBG_H_

void readKeyboard(movement_vector_t &movement_vector, states_t &state) {
  char incoming_byte = 0;

  //read the serial
  if (Serial.available() > 0) {
    incoming_byte = Serial.read();

    switch(incoming_byte) {
      case ' ': //stop line following
        resetMotorData(movement_vector);
        state = STOPPED;
        break;

      case 'F': //start
        state = FOLLOW_LINE;

        resetMotorData(movement_vector);

        LINE_SERIAL.write(LINE_SERIAL_RESET);
        delay(2000);
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

      case 'W':
        state = WAIT_FOR_LED;
        break;

      case 'I':
        state = INITIALIZE;
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

      case 'O':
        RUBI_SERIAL.write(RUBI_OPEN_ARMS);
        break;

      case 'C':
        RUBI_SERIAL.write(RUBI_CLOSE_ARMS);
        break;

      case 'M':
        RUBI_SERIAL.write(RUBI_RAISE);
        break;

      case 'k':
        RUBI_SERIAL.write('S');
        break;

      case 'j':
        RUBI_SERIAL.write('W');
        break;

      case 'l':
        RUBI_SERIAL.write(RUBI_ALIGN);
        break;

      case ';':
        RUBI_SERIAL.write('T');
        break;

      case ',':
        RUBI_SERIAL.write('Z');
        break;

      default:
        break;
    } //end switch
  } //end if
} //end readKeyboard();
#endif
