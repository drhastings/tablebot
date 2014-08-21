#! /bin/sh

make all && sudo avrdude -B1 -cusbtiny -pm328p -U flash:w:main.hex 
