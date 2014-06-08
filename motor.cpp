#include <math.h>
#include <Arduino.h>

void setup()
{
  pinMode(2, OUTPUT);
  pinMode(53, OUTPUT);
  digitalWrite(53,LOW);
  digitalWrite(52,LOW);
  Serial.begin(9600);
}
int main()
{
  char duty = 255;
  int counter = 0;
  int incomingByte = 0;
  init();
  setup();
  while (1)
  {
    analogWrite(2, duty);
    if (Serial.available() > 0)
    {
      incomingByte = Serial.read();

      if (incomingByte == ' ')
      {
        if (counter % 2 == 1)
        {
          digitalWrite(52,LOW);
          digitalWrite(53,LOW);
        }
        else
        {
          digitalWrite(53,LOW);
          digitalWrite(52,HIGH);
        }
        counter ++;
      }

      if (incomingByte == 65)
        duty += 2;
      else if (incomingByte == 66)
        duty -= 2;

      else if (incomingByte == 67)
      {
        digitalWrite(53,LOW);
        digitalWrite(52,HIGH);
        counter = 1;
      }

      else if (incomingByte == 68)
      {
        digitalWrite(52,LOW);
        digitalWrite(53,HIGH);
        counter = 1; //reset the counter so space to stop works correctly.
      }

    }
  }
  return 0;
}
