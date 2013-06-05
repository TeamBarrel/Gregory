#ifndef ADC_H
#define ADC_H

#include <htc.h>
#include "xtal.h"

void init_adc(void);
int adc_read(void);
int adc_read_channel(unsigned char channel);

#endif 