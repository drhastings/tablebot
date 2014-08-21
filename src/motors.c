#include <motors.h>
#include <avr/io.h>

void initMotors() 
{
  DDRB |= (1<<5) | (1<<3) | (1<<4);
  DDRD |= (1<<4) | (1<<5) | (1<<6);

  TCCR0A = (1<<COM0A1) | (1<<COM0B1) | (1<<WGM00);
  TCCR0B = (1<<CS00) | (1<<CS01);
}

void motora(int16_t power)
{
  if (power > 0)
  {
    if (power > 175) power = 175;

    PORTD = (PORTD | 0x10);
    PORTB = (PORTB & 0b11101111);
    OCR0A = power;
  }
  else if (power < 0)
  {
    if (power < -175) power = -175;
    PORTD = (PORTD & 0b11101111);
    PORTB = (PORTB | 0x10); 
    OCR0A = -power;
  }
  else
  {
    PORTB = (PORTB & 0b11101111);
    PORTD = (PORTD & 0b11101111);
    OCR0A = 255;
  }
}

void motorb(int16_t power)
{
  if (power < 0)
  {
    if (power < -175) power = -175;
    PORTB = (PORTB & ~(0x28)) | 0x20; 
    OCR0B = -power;
  }
  else if (power > 0)
  {
    if (power > 175) power = 175;
    PORTB = (PORTB & ~(0x28)) | 0x08;
    OCR0B = power;
  }
  else
  {
    PORTB = (PORTB & ~(0x28));
    OCR0B = 255;
  }
}
