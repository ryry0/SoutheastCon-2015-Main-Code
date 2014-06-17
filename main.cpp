//Author: Ryan - David Reyes
/*
 * The motor gear ratio is 30
 * The encoder resolution is 64
 * That makes 1920 ticks per shaft rev.
 *
 * The timer interrupt uses the following formula
 *
 * Motor Pinout:
 * Red    | motor power (connects to one motor terminal)
 * Black  | motor power (connects to the other motor terminal)
 * Green  | encoder GND
 * Blue   | encoder Vcc (3.5 - 20 V)
 * Yellow | encoder A output
 * White  | encoder B output
 *
 * (16M/prescaler)/(desired frequency) = number of counts for CTC mode.
 */
#define ARDUINO 102
#include <math.h>
#include <Arduino.h>
#include "Encoder.h"
#include "PID.h"

#define NO_PRESCALING 0x01
#define PRESCALE_8    0x02
#define PRESCALE_64   0x03

#define KP 1.0
#define KI 0
#define KD 0
//this is the number of ticks for CTC mode
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

#define ENCODER_RESOLUTION
#define GEAR_RATIO

#define ENCODER1_A 2
#define ENCODER1_B 3

#define ARROW_UP    65
#define ARROW_DOWN  66
#define ARROW_LEFT  67
#define ARROW_RIGHT 68

//structures
struct motor {
  char  pwm;
  char  pwm_pin;
  char  directiona;
  char  directionb;
  float command_velocity; //specified in rad/s
  float current_velocity; //specified in rad/s
};

//variables
motor    motor0;
pid_data motor0_pid_data;
Encoder  motor0_encoder(

//function prototypes
void setup();
void readKeyboard();
void timerInterrupt();

//interrupt handler for the timer compare
ISR(TIMER1_COMPA_vect) {

  fixedUpdatePID(motor0_pid_data, current_error);
}

//main
int main() {
  init();
  setup();

  while (1) {
    readKeyboard();
  }
  return 0;
} //end main()

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
  setPIDConstants(motor0_pid_data, KP, KI, KD);

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

  analogWrite(motor0.pwm_pin, duty);
  if (Serial.available() > 0) {
    incomingByte = Serial.read();

    switch(incomingByte) {
      case ' ':
        if (counter % 2 == 1) {
          digitalWrite(motor0.directiona,LOW);
          digitalWrite(motor0.directionb,LOW);
        }
        else {
          digitalWrite(motor0.directiona,LOW);
          digitalWrite(motor0.directionb,HIGH);
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
        digitalWrite(motor0.directiona,HIGH);
        digitalWrite(motor0.directionb,LOW);
        counter = 1;
        break;

      case ARROW_RIGHT:
        digitalWrite(motor0.directiona,LOW);
        digitalWrite(motor0.directionb,HIGH);
        counter = 1; //reset the counter so space to stop works correctly.
        break;
      default:
        break;
    }
  }
} //end readKeyboard();
