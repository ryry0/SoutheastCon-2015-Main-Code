//Author: Ryan - David Reyes
/*
 * The motor gear ratio is 30
 * The encoder resolution is 64
 * That makes 1920 ticks per shaft rev.
 *
 * The timer interrupt uses the following formula
 * (16M/prescaler)/(desired frequency) = number of counts for CTC mode.
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
#include "PID.h"
#include "motor.h"

#define NO_PRESCALING 0x01
#define PRESCALE_8    0x02
#define PRESCALE_64   0x03
//this is how many sample rate ticks it takes
//before the speed should be considered 0
#define TIME_THRESHOLD 25

#define PWM_SCALER (255/36.651)

#define TICKS_PER_REVOLUTION 1920

#define NUM_MOTORS 4

#define KP 2.269
#define KI 18.4417
#define KD 0
#define INT_GUARD 1000

//this is the number of ticks for CTC mode
#define SAMPLE_RATE 1000 //Hz
#define CTC_MATCH 16000 //*should* run the interrupt at 1kHz

//Motor 0 Pins
#define BACK_LEFT_MOTOR   0
#define MOTOR0_PWM_PINOUT 4
#define MOTOR0_DIRECTIONA 23 //I2
#define MOTOR0_DIRECTIONB 22 //I1
#define MOTOR0_ENCODER_A 2
#define MOTOR0_ENCODER_B 3

//Motor 1 Pins
#define FRONT_LEFT_MOTOR  1
#define MOTOR1_PWM_PINOUT 5
#define MOTOR1_DIRECTIONA 25 //I4
#define MOTOR1_DIRECTIONB 24 //I3
#define MOTOR1_ENCODER_A 2
#define MOTOR1_ENCODER_B 3

//Motor 2 Pins
#define FRONT_RIGHT_MOTOR 2
#define MOTOR2_PWM_PINOUT 6
#define MOTOR2_DIRECTIONA 51 //I2
#define MOTOR2_DIRECTIONB 50 //I1
#define MOTOR2_ENCODER_A 2
#define MOTOR2_ENCODER_B 3

//Motor 3 Pins
#define BACK_RIGHT_MOTOR  3
#define MOTOR3_PWM_PINOUT 7
#define MOTOR3_DIRECTIONA 53 //I4
#define MOTOR3_DIRECTIONB 52 //I3
#define MOTOR3_ENCODER_A 2
#define MOTOR3_ENCODER_B 3



#define ARROW_UP    65
#define ARROW_DOWN  66
#define ARROW_LEFT  67
#define ARROW_RIGHT 68


//variables
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

  for (int i = 0; i < NUM_MOTORS; ++i) {
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
    updatePID(motor_pid_data[i], current_error, .001);

    motors[i].pwm = motor_pid_data[i].pid_output;

    if (motors[i].pwm < 0) {
      motors[i].pwm = -motors[i].pwm;
      setMotorDirection(motors[i], DIRECTION_1);
    }
    else {
      setMotorDirection(motors[i], DIRECTION_2);
    }

    motors[i].pwm = constrain(motors[i].pwm, 0, 255);
    analogWrite(motors[i].pwm_pin, motors[i].pwm);
  }

  /*
  //profiling code
  time_now = micros();
  time_total = time_now - time_begin;

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

    //testing stuff
    if (motors[BACK_LEFT_MOTOR].current_velocity != prev_motor_velocity) {
      Serial.print(motors[BACK_LEFT_MOTOR].command_velocity, 4);
      Serial.print("\t");

      Serial.print(motors[BACK_LEFT_MOTOR].current_velocity, 4);
      Serial.print("\t");

      Serial.print(motor_pid_data[BACK_LEFT_MOTOR].previous_error, 4);
      Serial.print("\t");

      Serial.print(motor_pid_data[BACK_LEFT_MOTOR].pid_output, 4);
      Serial.print("\t");

      Serial.print(motors[BACK_LEFT_MOTOR].pwm, DEC);
      Serial.print("\n");

      prev_motor_velocity = motors[BACK_LEFT_MOTOR].current_velocity;
    }

    /*
       if (display_count > 100) {
       plot_time_now = millis();
       Serial.print(plot_time_now, 4);
       Serial.print("\t");

       Serial.print(motor0.command_velocity, 4);
       Serial.print("\t");

       Serial.print(motor0.current_velocity, 4);
       Serial.print("\t");

       Serial.print(avg_velocity, 4);
       Serial.print("\n");
       Serial.print(time_total, DEC);
       Serial.print("\n");

       display_count = 0;

       }

       if (motor0.encoder_value != prev_encoder_value) {
       prev_encoder_value = motor0.encoder_value;
       Serial.print(motor0.encoder_value, DEC);
       Serial.print("\n");
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

  //stop the motors
  for (int i = 0; i < NUM_MOTORS; ++i) {
    stopMotor(motors[i]);
  }

  //start the serial device
  Serial.begin(9600);

  //set the PID constants
  for (int i = 0; i < NUM_MOTORS; ++i) {
    setPIDConstants(motor_pid_data[i], KP, KI, KD, INT_GUARD);
  }

  //configure the timer interrupt
  TCCR1A = 0;
  TCCR1B = NO_PRESCALING; //sets the prescaler to 1
  TCNT1  = 0;             //resets the timer

  OCR1A = CTC_MATCH;
  TCCR1B |= (0x01 << WGM12);  //enables CTC mode
  TIMSK1 |= (0x01 << OCIE1A); //enables the interrupt CTC interrupt

  interrupts();
} //end setup()

void readKeyboard() {
  int incomingByte = 0;

  if (Serial.available() > 0) {
    incomingByte = Serial.read();

    switch(incomingByte) {
      case ' ':
        for (int i = 0; i < NUM_MOTORS; ++i) {
          motors[i].command_velocity = 0;
        }
        break;

      case ARROW_UP:
        for (int i = 0; i < NUM_MOTORS; ++i) {
          motors[i].command_velocity += 0.2;
        }
        break;

      case ARROW_DOWN:
        for (int i = 0; i < NUM_MOTORS; ++i) {
          motors[i].command_velocity -= 0.2;
        }
        break;

      case ARROW_LEFT:
        //moveMotor(motors[i], -1);
        for (int i = 0; i < NUM_MOTORS; ++i) {
          motors[i].command_velocity += 5.0;
        }
        break;

      case ARROW_RIGHT:
        //moveMotor(motors[i], 1);
        for (int i = 0; i < NUM_MOTORS; ++i) {
          motors[i].command_velocity -= 5.0;
        }
        break;

      case 'a':
        for (int i = 0; i < NUM_MOTORS; ++i) {
          motors[i].command_velocity = 18.0;
        }
        break;
      default:
        break;
    } //end switch
  } //end if
} //end readKeyboard();
