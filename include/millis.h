#ifndef MILLIS_H
#define MILLIS_H

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

volatile uint32_t ms;

void initMillis();

volatile uint8_t processMotors;

uint32_t millis();

void delay(uint16_t wait);

#endif
