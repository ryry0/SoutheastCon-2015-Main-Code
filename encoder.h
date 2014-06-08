#ifndef ENCODER_H_
#define ENCODER_H_

namespace encoder
{

unsigned long encoder_count;

uint8_t encoder_port_a;
uint8_t encoder_port_b;

uint8_t encoder_port_a_mask;
uint8_t encoder_port_b_mask;

bool a_old_state = 0;
bool b_new_state = 0;

void EncoderAInterrupt()
{
  b_new_state ^ a_old_state ? encoder_count++ : encoder_count --;
  //sets the state of encoder channel a
	(*portInputRegister(encoder_port_a) & encoder_port_a_mask) ?
	  a_old_state = 1 : a_old_state = 0;
}

void EncoderBInterrupt()
{
	(*portInputRegister(encoder_port_b) & encoder_port_b_mask) ?
	  b_new_state = 1 : b_new_state = 0;
  b_new_state ^ a_old_state ? encoder_count++ : encoder_count --;
}

void initEncoderChannelA(int pin_number)
{
	uint8_t timer = digitalPinToTimer(pin_number);
	encoder_port_a = digitalPinToPort(pin_number);
	encoder_port_a_mask = digitalPinToBitMask(pin_number);
	if (timer != NOT_ON_TIMER) turnOffPWM(timer);
	attachInterrupt(0, EncoderAInterrupt, CHANGE);
}

void initEncoderChannelB(int pin_number)
{
	uint8_t timer = digitalPinToTimer(pin_number);
	encoder_port_b = digitalPinToPort(pin_number);
	encoder_port_b_mask = digitalPinToBitMask(pin_number);
	if (timer != NOT_ON_TIMER) turnOffPWM(timer);
	attachInterrupt(1, EncoderBInterrupt, CHANGE);
}

}
#endif
