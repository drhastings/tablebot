#ifndef MOTORS_H
#define MOTORS_H

#include <inttypes.h>

#define MOTORB1 1 //Reverse
#define MOTORB2 2 //Forward
#define MOTORA1 3 //PB3
#define MOTORA2 4 //PB4
#define MOTORBE 2 //PB2
#define MOTORAE 1 //PB1

void initMotors();

void motora(int16_t power);
void motorb(int16_t power);

#endif
