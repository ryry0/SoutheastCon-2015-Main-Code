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

#define KP 2.269
#define KI 18.4417
#define KD 0
#define INT_GUARD 1000

//this is the number of ticks for CTC mode
#define SAMPLE_RATE 1000 //Hz
#define CTC_MATCH 16000 //*should* run the interrupt at 1kHz

//Motor 0 Pins
#define MOTOR0_PWM_PINOUT 5
#define MOTOR0_DIRECTIONA 24
#define MOTOR0_DIRECTIONB 25

//Motor 1 Pins
#define MOTOR1_PWM_PINOUT 2
#define MOTOR1_DIRECTIONA 53
#define MOTOR1_DIRECTIONB 52

//Motor 2 Pins
#define MOTOR2_PWM_PINOUT 3
#define MOTOR2_DIRECTIONA 40
#define MOTOR2_DIRECTIONB 41

//Motor 3 Pins
#define MOTOR3_PWM_PINOUT 4
#define MOTOR3_DIRECTIONA 22
#define MOTOR3_DIRECTIONB 23

#define ENCODER0_A 2
#define ENCODER0_B 3

#define ARROW_UP    65
#define ARROW_DOWN  66
#define ARROW_LEFT  67
#define ARROW_RIGHT 68


//variables
motor    motor0;
pid_data motor0_pid_data;
Encoder  motor0_encoder(ENCODER0_A, ENCODER0_B);
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
  static long time_since_last_tick = 1;

  //time_begin = micros();
  //get the current velocity in rads/s
  //to get the current velocity, we get the number of encoder ticks since the
  //last sample time, then we convert that into radians. Then we divide that by
  //the timestep.
  motor0.encoder_value = motor0_encoder.read();

  if (motor0.encoder_value != 0) {
    /*
     * current_velocity =           num_ticks * 2PI
     *                    __________________________________ * sample_rate
     *                    ticks/rev * millis_since_last_tick
     */
    motor0.current_velocity = ( ((float) motor0.encoder_value * 2*PI)/
        ((float) TICKS_PER_REVOLUTION * (float) time_since_last_tick) ) *
      (float) SAMPLE_RATE;

    /* averaging code
       velocity_samples[counter] = motor0.current_velocity;
       for (int i = 0; i < NUM_SAMPLES; ++i) {
         avg_velocity += velocity_samples[i];
       }
       avg_velocity = avg_velocity/NUM_SAMPLES;
       */

    motor0_encoder.write(0); //reset encoder count
    time_since_last_tick = 1;
  }
  else if (time_since_last_tick >= TIME_THRESHOLD) {
    motor0.current_velocity = 0.0; //so much time has passed we're not moving
  }
  else {
    ++time_since_last_tick;
  }

  //calculate PID
  current_error = motor0.command_velocity - motor0.current_velocity;
  updatePID(motor0_pid_data, current_error, .001);

  motor0.pwm = motor0_pid_data.pid_output;

  if (motor0.pwm < 0) {
    motor0.pwm = -motor0.pwm;
    moveMotor(motor0, -1);
  }
  else {
    moveMotor(motor0, 1);
  }

  motor0.pwm = constrain(motor0.pwm, 0, 255);
  analogWrite(motor0.pwm_pin, motor0.pwm);

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
    if (motor0.current_velocity != prev_motor_velocity) {
      Serial.print(motor0.command_velocity, 4);
      Serial.print("\t");

      Serial.print(motor0.current_velocity, 4);
      Serial.print("\t");

      Serial.print(motor0_pid_data.previous_error, 4);
      Serial.print("\t");

      Serial.print(motor0_pid_data.pid_output, 4);
      Serial.print("\t");

      Serial.print(motor0.pwm, DEC);
      Serial.print("\n");

      prev_motor_velocity = motor0.current_velocity;
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
  motor0.pwm = 0;
  motor0.pwm_pin = MOTOR0_PWM_PINOUT;
  motor0.directiona = MOTOR0_DIRECTIONA;
  motor0.directionb = MOTOR0_DIRECTIONB;
  motor0.command_velocity = 0;
  motor0.current_velocity = 0;

  //set all the pins to output
  pinMode(MOTOR0_PWM_PINOUT, OUTPUT);
  pinMode(MOTOR0_DIRECTIONA, OUTPUT);
  pinMode(MOTOR0_DIRECTIONB, OUTPUT);

  //stop the motor
  digitalWrite(motor0.directiona,LOW);
  digitalWrite(motor0.directionb,LOW);

  //start the serial device
  Serial.begin(9600);

  //set the PID constants
  setPIDConstants(motor0_pid_data, KP, KI, KD, INT_GUARD);

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
  static int counter = 0;
  int incomingByte = 0;

  if (Serial.available() > 0) {
    incomingByte = Serial.read();

    switch(incomingByte) {
      case ' ':
        if (counter % 2 == 1) {
          digitalWrite(motor0.directiona,LOW);
          digitalWrite(motor0.directionb,LOW);
          motor0.command_velocity = 0;
        }
        else {
          digitalWrite(motor0.directiona,HIGH);
          digitalWrite(motor0.directionb,LOW);
        }
        counter ++;
        break;

      case ARROW_UP:
        motor0.command_velocity += 0.2;
        break;

      case ARROW_DOWN:
        motor0.command_velocity -= 0.2;
        break;

      case ARROW_LEFT:
        //moveMotor(motor0, -1);
        motor0.command_velocity += 5.0;
        counter = 1;
        break;

      case ARROW_RIGHT:
        //moveMotor(motor0, 1);
        motor0.command_velocity -= 5.0;
        counter = 1; //reset the counter so space to stop works correctly.
        break;

      case 'a':
        motor0.command_velocity = 18.0;
        counter = 1; //reset the counter so space to stop works correctly.
        break;
      default:
        break;
    } //end switch
  } //end if
} //end readKeyboard();
