#include <math.h>
#include <Arduino.h>
#include "encoder.h"

#define PWM_PINOUT 4
#define MOTOR1_DIRECTIONA 53
#define MOTOR1_DIRECTIONB 52

#define ENCODER1_A 2
#define ENCODER1_B 3


void setup()
{
  pinMode(PWM_PINOUT, OUTPUT);
  pinMode(MOTOR1_DIRECTIONA, OUTPUT);

  digitalWrite(MOTOR1_DIRECTIONA,LOW);
  digitalWrite(MOTOR1_DIRECTIONB,LOW);

  Serial.begin(9600);

  encoder::initEncoderChannelA(ENCODER1_A);
  encoder::initEncoderChannelB(ENCODER1_B);
}

int main()
{
  char duty = 50;
  int counter = 0;
  int incomingByte = 0;
  unsigned long prev_enc_value = 1;

  init();
  setup();

  while (1)
  {
    analogWrite(PWM_PINOUT, duty);

    if (Serial.available() > 0)
    {
      incomingByte = Serial.read();

      if (incomingByte == ' ')
      {
        if (counter % 2 == 1)
        {
          digitalWrite(MOTOR1_DIRECTIONB,LOW);
          digitalWrite(MOTOR1_DIRECTIONA,LOW);
        }
        else
        {
          digitalWrite(MOTOR1_DIRECTIONA,LOW);
          digitalWrite(MOTOR1_DIRECTIONB,HIGH);
        }
        counter ++;
      }

      if (incomingByte == 65)
        duty += 2;
      else if (incomingByte == 66)
        duty -= 2;

      else if (incomingByte == 67)
      {
        digitalWrite(MOTOR1_DIRECTIONA,LOW);
        digitalWrite(MOTOR1_DIRECTIONB,HIGH);
        counter = 1;
      }

      else if (incomingByte == 68)
      {
        digitalWrite(MOTOR1_DIRECTIONB,LOW);
        digitalWrite(MOTOR1_DIRECTIONA,HIGH);
        counter = 1; //reset the counter so space to stop works correctly.
      }
    }

    if (encoder::encoder_count != prev_enc_value)
    {
      Serial.print(encoder::encoder_count, DEC);
      Serial.print("\n");
      prev_enc_value = encoder::encoder_count;
    }
  }
  return 0;
}
