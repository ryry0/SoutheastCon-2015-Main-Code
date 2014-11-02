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
#define LINE_RES_SCALE 0.01
//line response scaling. Takes the integer and scales it by this number

float x_vel = 0, y_vel = 0, ang_vel = 0;
const float inv_radius = 1.0/WHEEL_RADIUS; //inverse radius
const float pre_computed_LW2 = (LENGTH+WIDTH) / 2;

enum states_t  {STOPPED, FOLLOW_LINE};
states_t robot_state = STOPPED;

//motor variables
motor    motors[NUM_MOTORS];
pid_data motor_pid_data[NUM_MOTORS];
Encoder  motor_encoders[NUM_MOTORS] = {
  Encoder(MOTOR0_ENCODER_A, MOTOR0_ENCODER_B),
  Encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B),
  Encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B),
  Encoder(MOTOR3_ENCODER_A, MOTOR3_ENCODER_B)
};

/* test variables
unsigned long time_begin, time_now, time_total;
   unsigned long display_count = 0;
   const int NUM_SAMPLES = 10;
   float velocity_samples[NUM_SAMPLES] = {0};
   float avg_velocity = 0;
   */

//function prototypes
void setup();
void readKeyboard();
void readLineSensors();

//velocity computation functions
//
float computeVelocity(int wheelnum, const float &x_velocity,
    const float &y_velocity, const float &ang_velocity); //for wheel 0

//interrupt handler for the timer compare
ISR(TIMER1_COMPA_vect) {
  float current_error;
  //time_begin = micros();

  for (int i = 0; i < NUM_MOTORS; ++i) {
    motors[i].encoder_value = motor_encoders[i].read();

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
  float prev_motor_velocity = 0;
  long  prev_encoder_value = 0;
  long  plot_time_now, plot_time_prev;

  float prev_x_vel = 0, prev_y_vel = 0;

  states_t prev_state = STOPPED;

  init();
  setup();

  while (1) {
    readKeyboard();
    /*
    if (prev_encoder_value != motors[BACK_LEFT_MOTOR].encoder_value) {
      Serial.print("com val: ");
      Serial.print(motors[BACK_LEFT_MOTOR].command_position, 4);
      Serial.print(" pos val: ");
      Serial.print(motors[BACK_LEFT_MOTOR].encoder_value);
      Serial.print("\n");

      prev_encoder_value = motors[BACK_LEFT_MOTOR].encoder_value;
    }
    */
  }
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

void readKeyboard() {
  char incomingByte = 0;

  //read the serial
  if (Serial.available() > 0) {
    incomingByte = Serial.read();

    switch(incomingByte) {
      case ' ': //stop line following
        x_vel = 0;
        y_vel = 0;
        ang_vel = 0;
        for (int i = 0; i < NUM_MOTORS; ++i) {
          motors[i].pwm = 0;
          motors[i].command_position = 0;
          motors[i].encoder_value = 0;
          motor_encoders[i].write(0);
          motor_pid_data[i].pid_output = 0;
          motor_pid_data[i].previous_error = 0;
          motor_pid_data[i].integral_error = 0;
          stopMotor(motors[i]);
        }
        robot_state = STOPPED;
        break;

      case 'F': //start
        robot_state = FOLLOW_LINE;
        digitalWrite(RESET_PIN, LOW);
        for (int i = 0; i < 100; ++i);
        digitalWrite(RESET_PIN, HIGH);
        break;

      case 'q':
        for (int i = 0; i < ACTIVE_MOTORS; ++i) {
          motors[i].command_position += 50;
        }
        break;

      case 'a':
        for (int i = 0; i < ACTIVE_MOTORS; ++i) {
          motors[i].command_position -= 50;
        }
        break;

      case 'w':
        for (int i = 0; i < ACTIVE_MOTORS; ++i) {
          motors[i].command_position += 100;
        }
        break;

      case 's':
        for (int i = 0; i < ACTIVE_MOTORS; ++i) {
          motors[i].command_position -= 100;
        }
        break;

      case 'e':
        for (int i = 0; i < ACTIVE_MOTORS; ++i) {
          motors[i].command_position += 500;
        }
        break;

      case 'd':
        for (int i = 0; i < ACTIVE_MOTORS; ++i) {
          motors[i].command_position -= 500;
        }
        break;

      case ARROW_LEFT:
        ang_vel -= .3;
        break;
      case ARROW_RIGHT:
        ang_vel += .3;
        break;

      default:
        break;
    } //end switch
  } //end if
} //end readKeyboard();

//this deals with reading the slave arduino's line sensors
void readLineSensors() {
  char incomingByte = 0;
  //data comes in as [x or y][speed], so
  //this char tells me what came first
  static char velocity_id = 0;

  //read the serial
  if(robot_state == FOLLOW_LINE) {
    if (Serial3.available() > 0) {
      incomingByte = Serial3.read();

      switch(incomingByte) {
        case 'X':
        case 'Y':
        case 'B':
          //while(Serial3.available() == 0);//spin until next byte
          velocity_id = incomingByte;
          break;
        default:
          switch(velocity_id) {
            case 'X':
              x_vel = incomingByte * LINE_RES_SCALE;
              break;

            case 'Y':
              y_vel = incomingByte * LINE_RES_SCALE;
              break;

            default:
              break;
          }
          /*
          Serial.print(velocity_id);
          Serial.print("\t");
          Serial.print((int)incomingByte);
          Serial.print("\t");
          Serial.print(x_vel, 4);
          Serial.print("\t");
          Serial.print(y_vel, 4);
          Serial.print("\n");
          */
          velocity_id = 0;
          break;
      } //end switch
    } //end if
  }
} //end readLineSensors();

float computeVelocity(int wheelnum, const float &x_velocity,
    const float &y_velocity, const float &ang_velocity) { //for wheel 0
  float velocity = 0; //velocity of the wheel
  switch(wheelnum) {
    case 0:
        velocity = inv_radius * (x_velocity + y_velocity -
            pre_computed_LW2*ang_velocity);
      break;

    case 1:
        velocity = inv_radius * (x_velocity - y_velocity -
            pre_computed_LW2*ang_velocity);
      break;

    case 2:
        velocity = inv_radius * (x_velocity + y_velocity +
            pre_computed_LW2*ang_velocity);
      break;

    case 3:
        velocity = inv_radius * (x_velocity - y_velocity +
            pre_computed_LW2*ang_velocity);
      break;
  }
  return velocity;
}
