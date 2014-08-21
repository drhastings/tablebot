#include <millis.h>

void initMillis()
{
  TCCR1B |= 1<<WGM12 | 0<<CS12 | 0<<CS11 | 1<<CS10;
  OCR1A = 20000;
  TIMSK1 |= 1<<OCIE1A;
}

uint32_t millis()
{
	return ms;
}

void delay(uint16_t wait)
{
  uint32_t time = millis();
  while (time + wait > millis());
}

ISR (TIMER1_COMPA_vect)
{
  ms++;
}
