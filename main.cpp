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

#define KP 0.2 //15 //10 //2.2690
#define KI 25//18.4475
#define KD 0
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
  //static int counter = 0;
  float current_error;
  //time since last tick is used when the encoder creates pulses slower than
  //1 per millisecond
  static long time_since_last_tick[NUM_MOTORS] = {1, 1, 1, 1};

  //time_begin = micros();
  //get the current velocity in rads/s
  //to get the current velocity, we get the number of encoder ticks since the
  //last sample time, then we convert that into radians. Then we divide that by
  //the timestep.

  for (int i = 0; i < 4; ++i) {
    motors[i].encoder_value = motor_encoders[i].read();

    if (motors[i].encoder_value != 0) {
      /*
       * current_velocity =           num_ticks * 2PI
       *                    __________________________________ * sample_rate
       *                    ticks/rev * millis_since_last_tick
       */
      motors[i].current_velocity = ( ((float) motors[i].encoder_value * 2*PI)/
          ((float) TICKS_PER_REVOLUTION * (float) time_since_last_tick[i]) ) *
        (float) SAMPLE_RATE;

      /* averaging code
         velocity_samples[counter] = motor0.current_velocity;
         for (int i = 0; i < NUM_SAMPLES; ++i) {
         avg_velocity += velocity_samples[i];
         }
         avg_velocity = avg_velocity/NUM_SAMPLES;
         */

      motor_encoders[i].write(0); //reset encoder count
      time_since_last_tick[i] = 1;
    }
    else if (time_since_last_tick[i] >= TIME_THRESHOLD) {
      motors[i].current_velocity = 0.0; //so much time has passed we're not moving
    }
    else {
      ++time_since_last_tick[i];
    }

    //calculate PID
    current_error = motors[i].command_velocity - motors[i].current_velocity;
    updatePID(motor_pid_data[i], current_error, SAMPLE_TIME);

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

  //profiling code
  //time_now = micros();
  //time_total = time_now - time_begin;

  /*
  ++counter;
  counter = counter % NUM_SAMPLES;
  ++display_count;
  */
} //end interrupt handler

//main
int main() {
  float prev_motor_velocity = 0;
  long  prev_encoder_value = 0;
  long  plot_time_now, plot_time_prev;

  init();
  setup();

  while (1) {
    readKeyboard();
    readLineSensors();

    for (int i = 0; i < NUM_MOTORS; ++i) { //compute motors on every iteration
      motors[i].command_velocity = computeVelocity(i, x_vel, y_vel, ang_vel);
    }
    if (robot_state == FOLLOW_LINE) {
      Serial.print("LINE");
    }
    else {
      Serial.print("STOP");
    }
    Serial.print("\tx_vel: ");
    Serial.print(x_vel, 4);
    Serial.print("\t");

    Serial.print("y_vel: ");
    Serial.print(y_vel, 4);
    /*
    Serial.print("\t");
    for (int i = 0; i < NUM_MOTORS; ++i) {
      Serial.print(motors[i].command_velocity, 4);
      Serial.print("\t");
    }
    */
    Serial.print("\n");
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

  //start the serial device
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
        robot_state = STOPPED;
        break;

      case 'F': //start
        robot_state = FOLLOW_LINE;
        digitalWrite(RESET_PIN, LOW);
        for (int i = 0; i < 100; ++i);
        digitalWrite(RESET_PIN, HIGH);
        break;

      case 'w':
        x_vel += 0.1;
        break;

      case 's':
        x_vel -= 0.1;
        break;

      case 'a':
        y_vel -= 0.1;
        break;

      case 'd':
        y_vel += 0.1;
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
