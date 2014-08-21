#include "../include/uart.h"

typedef struct
{
  uint8_t sending;
  uint8_t index;
  uint8_t length;
  uint8_t inIndex;
  uint8_t inLength;
  uint8_t incoming;
} statusReg;

volatile statusReg reg;
volatile char buffer[256];
volatile char inBuffer[256];

void initUART()
{
  reg.sending = 0;
  reg.index = 0;
  reg.length = 0;
  reg.inIndex = 0;
  reg.inLength = 0;
  reg.incoming = 0;
  UBRR0H = 0;
  UBRR0L = (unsigned char)129;
  UCSR0B = 1<<RXEN0 | 1<<TXEN0 | 1<<TXCIE0 | (1<<RXCIE0);
  UCSR0C = 0<<USBS0 | 3<<UCSZ00;
}

uint8_t send(char * message, uint8_t length)
{
  uint8_t x;
  for (x = 0; x < length; x++)
  {
    buffer[(uint8_t) x + reg.length] = message[x];
  }
//  reg.index = 0;
  reg.length += length;
  if (reg.sending == 0)
  {
    UDR0 = buffer[reg.index];
    reg.index++;
  }
  reg.sending = 1;
  return 0;
}

uint8_t sendInt(int32_t number)
{
  uint8_t length = 0;
  char * message;

  while (pow(10, length) <= abs(number)) length++;

  if (number <= 0) length++;

  message = malloc(sizeof(char) * length);

  ltoa(number, message, 10);

  send(message, length);

  free(message);

  return 0;
}

uint8_t sendFloat(float number)
{
  char message[20];

  int8_t length = sprintf(message, "%.8f", number);

  send(message,length);
  
  free(message);

  return 0;
}

ISR (USART_TX_vect)
{
if (reg.sending)
  {
    if (reg.length == reg.index)
    {
      reg.sending = 0;
    }
    else
    {
      UDR0 = buffer[reg.index];
      reg.index++;
    }
  }
}

uint8_t available()
{
  return reg.inLength - reg.inIndex;
}

void flush()
{
  reg.inIndex - reg.inLength;
  reg.incoming = 0;
}

uint8_t getChar(char * character)
{
  if (reg.incoming)
  {
    *character = inBuffer[reg.inIndex];
    reg.inIndex++;
    if (reg.inIndex == reg.inLength) reg.incoming = 0;
    return 1;
  }
  else
  {
    return 0;
  }
}

uint8_t getLine(char * message)
{
  if(reg.incoming)
  {
    uint8_t x = 0;
    getChar(message);
    while (message[x] != 0x0A)
    {
      x++;
      getChar(message + x);
    }
    return x;
  }
  else
  {
    return 0;
  }
}

ISR (USART_RX_vect)
{
  inBuffer[reg.inLength] = UDR0;
  reg.inLength++;
  reg.incoming = 1;
}
