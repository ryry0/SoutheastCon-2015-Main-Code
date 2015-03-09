//Author: Ryan - David Reyes
/*
 * The motor gear ratio is 30
 * The encoder resolution is 64
 * That makes 1920 ticks per shaft rev.
 *
 * The timer interrupt uses the following formula
 * (16M/prescaler)/(desired frequency) = number of counts for CTC mode.
 * (16M/8)/(200) = CTC_MATCH = 10000
 *
 * Motor Pinout:
 * Red    | motor power (connects to one motor terminal)
 * Black  | motor power (connects to the other motor terminal)
 * Green  | encoder GND
 * Blue   | encoder Vcc (3.5 - 20 V)
 * Yellow | encoder A output
 * White  | encoder B output
 *
 * The pololu motor max speed is 350 rev/minute.
 * In radians per second that is:
 * 350/60 * 2pi = 36.651 rad/s @ theoretically 255 PWM
 */

#include "math.h"
#include "Arduino.h"

#define ARDUINO 102
#define ENCODER_USE_INTERRUPTS
#define SERIAL_DEBUG
#define KBD_DEBUG

#include "motor.h"
#include "PID.h"
#include "motor_pins.h"

#define RESET_PIN 48

#define NO_PRESCALING 0x01
#define PRESCALE_8    0x02
#define PRESCALE_64   0x03

#define TICKS_PER_REVOLUTION 1920

#define NUM_MOTORS 4

//original kp .7 and kd 10 small since not scaled by time
//10 and 50
#define KP 10//.7 //1//15 //10 //2.2690
#define KI 0 //18.4475
#define KD 50 //0.01
#define INT_GUARD 1000

//this is the number of ticks for CTC mode
#define SAMPLE_RATE 200 //Hz
#define CTC_MATCH 10000 //*should* run the interrupt at 200Hz
#define SAMPLE_TIME 0.005

#define GAME_TIMEOUT 25000 //time to wait for the game to complete in ms
#define TIMEOUT      5000

//define serial to use for each subsystem
#define LINE_SERIAL Serial2
#define ETCH_SERIAL Serial1
#define RUBI_SERIAL

//Line following defines
#define LINE_SERIAL_START 'S'
#define LINE_SERIAL_RESET 'R'

#define PACKET_LENGTH 8 //header cobs_byte y x a g d1 d2
#define LINE_PACKET_HEADER 0x00

//Etch-A-Sketch defines
#define ETCH_OPEN_ARMS  'O'
#define ETCH_CLOSE_ARMS 'C'
#define ETCH_PLAY_GAME  'P'

//robot specifications
#define WHEEL_RADIUS 0.0508 //[m]
#define LENGTH 0.115 //0.1 //[m] length of chassis from front to back
#define WIDTH 0.25 //0.19 //[m] width of chassis from left to right
#define LINE_RES_SCALE 0.01
//line response scaling. Takes the integer and scales it by this number

const float inv_radius = 1.0/WHEEL_RADIUS; //inverse radius
const float pre_computed_LW2 = (LENGTH+WIDTH) / 2;

// Photoresistor things
const float prk  = 1.0/5.0;
#define PHOTORESISTOR_THRESHOLD 400
#define PHOTORESISTOR           A0

enum states_t  {  STOPPED,        //default state when powered on
                  WAIT_FOR_LED,   //game started: wait for the led to turn on
                  INITIALIZE,     //LED OFF, robot initializes arm positions
                  FOLLOW_LINE,    //Follows the line
                  ETCH_A_SKETCH,  //Plays etch a sketch
                  RUBIKS,         //Plays rubik's
                  SIMON,          //Plays simon
                  CARD,           //Plays card
                  WAIT_FOR_GAME,  //Waits till game is done
                  FINISH,         //State of robot @ finish line. Retracts arms
                  DBG_LINE_SENSORS}; //Get line sensor data w/o moving

//data struct definitions
//movement vector is heading of robot in m/s
struct movement_vector_t {
  float x_velocity;
  float y_velocity;
  float angular_velocity;
};

//line following packet represents the //data sent by line MCU.
struct line_following_packet_t {
  char y_velocity;
  char x_velocity;
  char angular_velocity;
  char game_state;
  char debug1;
  char debug2;
};

//global motor variables
motor    motors[NUM_MOTORS]; //struct of variables dealing with motors
pid_data motor_pid_data[NUM_MOTORS]; //struct of data dealing with PID
volatile long motor_encoders[NUM_MOTORS];

//function prototypes
//resets all data pertaining to motors
void resetMotorData(movement_vector_t &movement_vector);
#ifdef KBD_DEBUG
#include "kbd_dbg.h" //keyboard debugging function
#endif
void setup();
bool readLineSensors(line_following_packet_t &line_packet);

//velocity computation function
float computeVelocity(const int wheelnum,
    const movement_vector_t &movement_vector);

//port K interrupt vector
ISR(PCINT2_vect) {
  static const int8_t rot_states[] = //lookup table of rotation states
  {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
  static uint8_t  AB[NUM_MOTORS] = {0x03, 0x03, 0x03, 0x03};
  uint8_t         t = PINK;  // read port status

  for (int i = 0; i < NUM_MOTORS; ++i) {
    // check for rotary state change
    AB[i] <<= 2;                  // save previous state
    AB[i] |= (t >> 2*i) & 0x03;     // add current state
    motor_encoders[i] += rot_states[AB[i] & 0x0f];
  }
}

//interrupt handler for the timer compare
ISR(TIMER1_COMPA_vect) {
  float current_error;

  for (int i = 0; i < NUM_MOTORS; ++i) {
    motors[i].encoder_value = motor_encoders[i];

    //calculate new position based on velocity
    motors[i].command_position += SAMPLE_TIME * motors[i].command_velocity;

    //calculate PID
    current_error = motors[i].command_position - motors[i].encoder_value;
    fixedUpdatePID(motor_pid_data[i], current_error);

    //pwm is absolute value of output
    motors[i].pwm = round(fabs(motor_pid_data[i].pid_output));
    motors[i].pwm = constrain(motors[i].pwm, 0, 255);

    //if output is < 0 switch directions
    if (motor_pid_data[i].pid_output < 0)
      setMotorDirection(motors[i], DIRECTION_1);
    else
      setMotorDirection(motors[i], DIRECTION_2);

    //write to pwm
    analogWrite(motors[i].pwm_pin, motors[i].pwm);
  }
} //end interrupt handler

//main
int main() {
  //variables for determining robot direction
  states_t                robot_state = WAIT_FOR_LED; //overall state of the robot
  movement_vector_t       movement_vector = {0};
  line_following_packet_t line_packet = {0}; //line packet stuff

  //photoresistor voltage
  float pr_voltage = 0;

  //timing stuff
  unsigned long start_time = 0;

#ifdef SERIAL_DEBUG
  //These variables hold the previous values for debugging purposes
  unsigned char prev_debug1 = 0, prev_debug2 = 0;
  states_t prev_state = STOPPED;
  float prev_x_vel = 0, prev_y_vel = 0, prev_ang_vel = 0;
#endif

  init();
  setup();

  //main loop
  while (1) {
    //state machine
    switch(robot_state) {
      case STOPPED:
        //silence motors
        for (int i = 0; i < NUM_MOTORS; ++i) {
          if (motors[i].command_velocity == 0) {
            motors[i].command_position = motor_encoders[i];
          }
        } //end for
        break; //end stopped

      case WAIT_FOR_LED:
        // Wait for LED code
        // Analog measurement is connected across PR (5k-100k)
        // with 50k resistor in series.
        // Once the light goes off, 2/3 of the 5V should be across the PR,
        // so threshold is 512
        pr_voltage = (1-prk)*pr_voltage + prk*(float)analogRead(PHOTORESISTOR);
        if(pr_voltage > PHOTORESISTOR_THRESHOLD) robot_state = INITIALIZE;
        break; //end wait for led

      case INITIALIZE:
        //clears all data
        resetMotorData(movement_vector);

        //opens arms before beginning of run
        ETCH_SERIAL.write(ETCH_OPEN_ARMS);

        //reset the serial
        LINE_SERIAL.write(LINE_SERIAL_START);

        line_packet.game_state = 0;
        robot_state = FOLLOW_LINE;
        break; //end initialize

      case FOLLOW_LINE:
        //read line sensors will ask for data when nothing is on the line,
        //or when the first byte it reads is not 0xFF
        if (readLineSensors(line_packet))
          LINE_SERIAL.write(' ');

        //Movement assignments
        movement_vector.x_velocity = line_packet.x_velocity * LINE_RES_SCALE;
        movement_vector.y_velocity = line_packet.y_velocity * LINE_RES_SCALE;
        movement_vector.angular_velocity =
          line_packet.angular_velocity * LINE_RES_SCALE;

        switch(line_packet.game_state) {
          case 'E':
            robot_state = ETCH_A_SKETCH;
            break;

          case 'C':
            robot_state = CARD;
            break;

          case 'S':
            robot_state = SIMON;
            break;

          case 'R':
            robot_state = RUBIKS;
            break;

          case 'F':
            robot_state = FINISH;
            break;

          default:
            break;
        } //end switch (line_packet.game_state);

        //make sure to set the start time
        if (line_packet.game_state != 0)
          start_time = millis();

        break; //END FOLLOW LINE

      //Gameplay states
      case ETCH_A_SKETCH:
        /* Etch A Sketch code goes here */
        ETCH_SERIAL.write(ETCH_PLAY_GAME);

        robot_state = WAIT_FOR_GAME;
        break; //end etch a sketch

      case RUBIKS:
      case SIMON:
      case CARD:
        if ((millis() - start_time ) > TIMEOUT) {
          robot_state = FOLLOW_LINE;
          line_packet.game_state = 0;
        }
        break;

      case WAIT_FOR_GAME:
        //timer for timing out the game
        if ((millis() - start_time ) > GAME_TIMEOUT) {
          robot_state = FOLLOW_LINE;
          line_packet.game_state = 0;
        }
        break;

      case DBG_LINE_SENSORS:
        //read line sensors will ask for data when nothing is on the line,
        //or when the first byte it reads is not 0xFF
        if (readLineSensors(line_packet))
          LINE_SERIAL.write(' ');
        movement_vector.x_velocity = 0;
        movement_vector.y_velocity = 0;
        movement_vector.angular_velocity = 0;
        for (int i = 0; i < NUM_MOTORS; ++i) {
          motors[i].command_velocity = 0;
        }

        break; //END DBG_LINE_SENSORS

        //finish case
      case FINISH:
        ETCH_SERIAL.write(ETCH_CLOSE_ARMS);
        robot_state = STOPPED;
        break; //end finish

      default:
        break;
    } //end state switch

    //compute the velocity for each motor
    for (int i = 0; i < NUM_MOTORS; ++i) {
      motors[i].command_velocity =
        (960*computeVelocity(i, movement_vector))/PI;
    }//scale the velocity into ticks per second

    //DEBUGGING CODE
#ifdef KBD_DEBUG
    readKeyboard(movement_vector, robot_state);
#endif

#ifdef SERIAL_DEBUG
    if ((prev_state != robot_state) ||
        (prev_x_vel != movement_vector.x_velocity) ||
        (prev_y_vel != movement_vector.y_velocity) ||
        (prev_ang_vel != movement_vector.angular_velocity) ||
        (prev_debug1 != line_packet.debug1) ||
        (prev_debug2 != line_packet.debug2)) {

      if (robot_state == STOPPED)
        Serial.print("STOP");
      else
        Serial.print(line_packet.game_state);

      Serial.print("\tx_vel: ");
      Serial.print(movement_vector.x_velocity, 4);
      Serial.print("\t");

      Serial.print("\ty_vel: ");
      Serial.print(movement_vector.y_velocity, 4);

      Serial.print("\tang_vel: ");
      Serial.print(movement_vector.angular_velocity, 4);
      //Serial.print("\n");

      Serial.print(" F");
      Serial.write(line_packet.debug1);
      Serial.print("B");
      Serial.write(line_packet.debug2);
      Serial.print("\n");

      prev_x_vel = movement_vector.x_velocity;
      prev_y_vel = movement_vector.y_velocity;
      prev_ang_vel = movement_vector.angular_velocity;
      prev_state = robot_state;
      prev_debug1 = line_packet.debug1;
      prev_debug2 = line_packet.debug2;
    } //end if
#endif
    //END DEBUGGING CODE
  } //end while
  return 0;
} //end main()

//function definitions
void setup() {
  noInterrupts();
  //set up the motors
  for (int i = 0; i < NUM_MOTORS; ++i) {
    motors[i].pwm = 0;
    motors[i].command_velocity = 0;
    motors[i].current_velocity = 0;
  }

  motors[BACK_LEFT_MOTOR].pwm_pin = MOTOR0_PWM_PINOUT;
  motors[BACK_LEFT_MOTOR].directiona = MOTOR0_DIRECTIONA;
  motors[BACK_LEFT_MOTOR].directionb = MOTOR0_DIRECTIONB;

  motors[FRONT_LEFT_MOTOR].pwm_pin = MOTOR1_PWM_PINOUT;
  motors[FRONT_LEFT_MOTOR].directiona = MOTOR1_DIRECTIONA;
  motors[FRONT_LEFT_MOTOR].directionb = MOTOR1_DIRECTIONB;

  motors[FRONT_RIGHT_MOTOR].pwm_pin = MOTOR2_PWM_PINOUT;
  motors[FRONT_RIGHT_MOTOR].directiona = MOTOR2_DIRECTIONA;
  motors[FRONT_RIGHT_MOTOR].directionb = MOTOR2_DIRECTIONB;

  motors[BACK_RIGHT_MOTOR].pwm_pin = MOTOR3_PWM_PINOUT;
  motors[BACK_RIGHT_MOTOR].directiona = MOTOR3_DIRECTIONA;
  motors[BACK_RIGHT_MOTOR].directionb = MOTOR3_DIRECTIONB;

  //set all the pins to output
  for (int i = 0; i < NUM_MOTORS; ++i) {
    pinMode(motors[i].pwm_pin, OUTPUT);
    pinMode(motors[i].directiona, OUTPUT);
    pinMode(motors[i].directionb, OUTPUT);
  }

  pinMode(RESET_PIN, OUTPUT);

  digitalWrite(RESET_PIN, LOW);

  //stop the motors
  for (int i = 0; i < NUM_MOTORS; ++i) {
    stopMotor(motors[i]);
  }

  //start the serial devices
  Serial.begin(9600);
  LINE_SERIAL.begin(9600);
  ETCH_SERIAL.begin(9600);

  //set the PID constants
  for (int i = 0; i < NUM_MOTORS; ++i) {
    setPIDConstants(motor_pid_data[i], KP, KI, KD, INT_GUARD);
  }

  //configure the timer interrupt
  TCCR1A = 0;
  TCCR1B = PRESCALE_8; //sets the prescaler to 1
  TCNT1  = 0;             //resets the timer

  OCR1A = CTC_MATCH;
  TCCR1B |= (0x01 << WGM12);  //enables CTC mode
  TIMSK1 |= (0x01 << OCIE1A); //enables the interrupt CTC interrupt

  /////////////////////////////////////////////////////
  //set encoder pins to input
  DDRK = 0x00;

  // enable pullup for encoder pins
  PORTK |= _BV(PORTK7) | _BV(PORTK6) | _BV(PORTK5) |
    _BV(PORTK4) | _BV(PORTK3) | _BV(PORTK2) |
    _BV(PORTK1) | _BV(PORTK0);

  // enable button pin change interrupt
  PCMSK2 = _BV(PCINT16) | _BV(PCINT17) | _BV(PCINT18) | _BV(PCINT19) |
    _BV(PCINT20) | _BV(PCINT21) | _BV(PCINT22) | _BV(PCINT23);
  PCICR = _BV(PCIE2);  // K-port interrupt enable
  /////////////////////////////////////////////////////

  interrupts();
} //end setup()

//this deals with reading the slave arduino's line sensors
//readLineSensors returns whether or not the Request packet should be resent
//non-blocking because you can't assume data always comes.
//Now it also performs on the fly COBS decoding
bool readLineSensors(line_following_packet_t &line_packet) {
  char                  incoming_byte = 0;
  char                  *packet_ptr = (char *) &line_packet;
  static unsigned char  bytes_read = 0,
                        next_zero_byte_pos = 0,
                        current_zero_byte_pos = 1;
  bool                  request_packet = false; //if received the correct header
  //first byte read should be 0xFF, then the rest are y, x, o, i.

  //read the serial
  if (LINE_SERIAL.available() > 0) {
    incoming_byte = LINE_SERIAL.read();

    if (bytes_read == 0) { //check for header but don't store it
      if ((unsigned char) incoming_byte != LINE_PACKET_HEADER)
        request_packet = true;
    }

    else if (bytes_read == 1) {
      next_zero_byte_pos = incoming_byte; //get the code byte
      current_zero_byte_pos = 1;
    }

    else if (bytes_read > 1) { //perform the decoding as it comes
      //minus 2 because packet struct does not contain header byte or cobs_byte
      if (bytes_read - current_zero_byte_pos == next_zero_byte_pos) {
        next_zero_byte_pos = incoming_byte;
        current_zero_byte_pos = bytes_read;
        *(packet_ptr + bytes_read - 2) = 0;
      }
      else
        *(packet_ptr + bytes_read - 2) = incoming_byte;
    } //end else if (bytes_read > 1)

    bytes_read++;
    if (bytes_read >= PACKET_LENGTH) //reset bytes_read
      bytes_read = 0;

    if (request_packet == true) //reset bytes_read when resending packet
      bytes_read = 0;
  } //end if (LINE_SERIAL.available() > 0)
  else {
    request_packet = true;
  }

  return request_packet; //return whether we need to resend the packet or not
} //end readLineSensors();

float computeVelocity(const int wheelnum,
    const movement_vector_t &movement_vector) {
  float velocity = 0; //velocity of the wheel
  switch(wheelnum) {
    case 0:
      velocity = inv_radius * (movement_vector.x_velocity +
          movement_vector.y_velocity -
          pre_computed_LW2*movement_vector.angular_velocity);
      break;

    case 1:
      velocity = inv_radius * (movement_vector.x_velocity -
          movement_vector.y_velocity -
          pre_computed_LW2*movement_vector.angular_velocity);
      break;

    case 2:
      velocity = inv_radius * (movement_vector.x_velocity +
          movement_vector.y_velocity +
          pre_computed_LW2*movement_vector.angular_velocity);
      break;

    case 3:
      velocity = inv_radius * (movement_vector.x_velocity -
          movement_vector.y_velocity +
          pre_computed_LW2*movement_vector.angular_velocity);
      break;
  }
  return velocity;
}

void resetMotorData(movement_vector_t &movement_vector) {
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
  } // end for
} //end resetMotorData
