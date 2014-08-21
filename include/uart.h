#ifndef UART_H
#define UART_H

#include <avr/interrupt.h>
#include <stdlib.h>
#include <math.h>

#define sendLn() send("\n",1)

void initUART();

uint8_t send(char * message, uint8_t length);
uint8_t sendInt(int32_t number);
uint8_t sendFloat(float number);

uint8_t available();

void flush();

uint8_t getChar(char * character);

uint8_t getLine(char * message);

#endif
