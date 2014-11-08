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

#include <math.h>
#include <Arduino.h>

#define ARDUINO 102
#define ENCODER_USE_INTERRUPTS
#define SERIAL_DEBUG

#include "Encoder.h"
#include "motor.h"
#include "PID.h"
#include "motor_pins.h"

#define RESET_PIN 33
#define NO_PRESCALING 0x01
#define PRESCALE_8    0x02
#define PRESCALE_64   0x03
//this is how many sample rate ticks it takes
//before the speed should be considered 0
#define TIME_THRESHOLD 25

#define PWM_SCALER (255/36.651)

#define TICKS_PER_REVOLUTION 1920

#define NUM_MOTORS 4
#define ACTIVE_MOTORS 4

//small since not scaled by time
#define KP .7 //1//15 //10 //2.2690
#define KI 0 //18.4475
#define KD 10 //0.01
#define INT_GUARD 1000

//this is the number of ticks for CTC mode
#define SAMPLE_RATE 200 //Hz
#define CTC_MATCH 10000 //*should* run the interrupt at 200Hz
#define SAMPLE_TIME 0.005

#define ARROW_UP    65
#define ARROW_DOWN  66
#define ARROW_LEFT  67
#define ARROW_RIGHT 68

//robot specifications
#define WHEEL_RADIUS 0.0508 //[m]
#define LENGTH 0.1 //[m] length of chassis from front to back
#define WIDTH 0.19 //[m] width of chassis from left to right
#define LINE_RES_SCALE 1 //0.002
//line response scaling. Takes the integer and scales it by this number

const float inv_radius = 1.0/WHEEL_RADIUS; //inverse radius
const float pre_computed_LW2 = (LENGTH+WIDTH) / 2;

enum states_t  {STOPPED, FOLLOW_LINE};

struct movement_vector_t {
  float x_velocity;
  float y_velocity;
  float angular_velocity;
};

//variables that hold debugging data about line following sensors
unsigned char outer_sensors = 0, inner_sensor = 0;

//global motor variables
motor    motors[NUM_MOTORS]; //struct of variables dealing with motors
pid_data motor_pid_data[NUM_MOTORS]; //struct of data dealing with PID
Encoder  motor_encoders[NUM_MOTORS] = {
  Encoder(MOTOR0_ENCODER_A, MOTOR0_ENCODER_B),
  Encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B),
  Encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B),
  Encoder(MOTOR3_ENCODER_A, MOTOR3_ENCODER_B)
};

//function prototypes
void setup();
void readKeyboard(movement_vector_t &movement_vector, states_t &state);
void readLineSensors(movement_vector_t &movement_vector);

//velocity computation functions
float computeVelocity(const int wheelnum,
    const movement_vector_t &movement_vector);

//interrupt handler for the timer compare
ISR(TIMER1_COMPA_vect) {
  float current_error;

  for (int i = 0; i < NUM_MOTORS; ++i) {
    motors[i].encoder_value = motor_encoders[i].read();

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
  movement_vector_t movement_vector;
  movement_vector.x_velocity = 0;
  movement_vector.y_velocity = 0;
  movement_vector.angular_velocity = 0;
  //overall state of the robot
  states_t robot_state = STOPPED;

#ifdef SERIAL_DEBUG
  /*
     These variables hold the previous values for debugging
     purposes
     float prev_motor_velocity = 0;
     long  prev_encoder_value = 0;
     */
  unsigned char prev_inner_sensor = 0, prev_outer_sensors = 0;
  states_t prev_state = STOPPED;
  float prev_x_vel = 0, prev_y_vel = 0;
#endif

  init();
  setup();

  //main loop
  while (1) {
    //state machine
    switch(robot_state) {
      case FOLLOW_LINE:
        readLineSensors(movement_vector);
        break;

      default:
        break;
    } //end state switch

    //compute the velocity for each motor
    for (int i = 0; i < NUM_MOTORS; ++i) {
      motors[i].command_velocity =
        (960*computeVelocity(i, movement_vector))/PI;
    }//scale the velocity into ticks per second

    //DEBUGGING CODE
#ifdef SERIAL_DEBUG
    readKeyboard(movement_vector, robot_state);
    if ((prev_state != robot_state) ||
        (prev_x_vel != movement_vector.x_velocity) ||
        (prev_y_vel != movement_vector.y_velocity) ||
        (prev_inner_sensor != inner_sensor) ||
        (prev_outer_sensors != outer_sensors )) {

      if (robot_state == FOLLOW_LINE)
        Serial.print("LINE");
      else
        Serial.print("STOP");

      Serial.print("\tx_vel: ");
      Serial.print(movement_vector.x_velocity, 4);
      Serial.print("\t");

      Serial.print("\ty_vel: ");
      Serial.print(movement_vector.y_velocity, 4);
      //Serial.print("\n");

      Serial.print(" O ");
      Serial.print(outer_sensors);
      Serial.print(" I ");
      Serial.print(inner_sensor);
      Serial.print("\n");

      prev_x_vel = movement_vector.x_velocity;
      prev_y_vel = movement_vector.y_velocity;
      prev_state = robot_state;
      prev_inner_sensor = inner_sensor;
      prev_outer_sensors = outer_sensors;
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
  Serial3.begin(9600);

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

  interrupts();
} //end setup()

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
          motor_encoders[i].write(0);
          motor_pid_data[i].pid_output = 0;
          motor_pid_data[i].previous_error = 0;
          motor_pid_data[i].integral_error = 0;
          stopMotor(motors[i]);
        }
        state = STOPPED;
        break;

      case 'F': //start
        state = FOLLOW_LINE;
        digitalWrite(RESET_PIN, LOW);
        for (int i = 0; i < 100; ++i);
        digitalWrite(RESET_PIN, HIGH);
        break;

      case 'w':
        movement_vector.x_velocity += 0.1;
        break;

      case 's':
        movement_vector.x_velocity -= 0.1;
        break;

      case 'a':
        movement_vector.y_velocity -= 0.1;
        break;

      case 'd':
        movement_vector.y_velocity += 0.1;
        break;

      case 'e':
        movement_vector.angular_velocity -= .3;
        break;

      case 'q':
        movement_vector.angular_velocity += .3;
        break;

      default:
        break;
    } //end switch
  } //end if
} //end readKeyboard();

//this deals with reading the slave arduino's line sensors
void readLineSensors(movement_vector_t &movement_vector) {
  char incoming_byte = 0;
  //data comes in as [x or y][speed], so
  //this char tells me what came first
  static char velocity_id = 0;

  //read the serial
  if (Serial3.available() > 0) {
    incoming_byte = Serial3.read();

    switch(incoming_byte) {
      case 'X':
      case 'Y':
      case 'B':
      case 'O':
      case 'I':
        //while(Serial3.available() == 0);//spin until next byte
        velocity_id = incoming_byte;
        break;
      default:
        switch(velocity_id) {
          case 'X':
            movement_vector.x_velocity = incoming_byte * LINE_RES_SCALE;
            break;

          case 'Y':
            movement_vector.y_velocity = incoming_byte * LINE_RES_SCALE;
            break;

          case 'O':
            outer_sensors = incoming_byte;
            break;

          case 'I':
            inner_sensor = incoming_byte;
            break;

          default:
            break;
        }
        velocity_id = 0; //reset velocity ID
        break;
    } //end switch
  }
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
