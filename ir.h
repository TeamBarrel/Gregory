#ifndef IR_H
#define IR_H

#include <htc.h>
#include "xtal.h"
#include "adc.h"

int convert(int adc_value);
int readIR();
void rotateIR(char steps, char direction);

#endif 
