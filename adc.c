#define BUILD // Comment this line out to omit from build
#if defined(BUILD)

#include "adc.h"

int adc_read_channel(unsigned char channel)
{
	switch(channel)
	{
		case 0: 
				CHS0 = 0;
				CHS1 = 0;
				CHS2 = 0;
				break;
		case 1: 
				CHS0 = 1;
				CHS1 = 0;
				CHS2 = 0;
				break;
		case 2: 
				CHS0 = 0;
				CHS1 = 1;
				CHS2 = 0;
				break;
		case 3: 
				CHS0 = 1;
				CHS1 = 1;
				CHS2 = 0;
				break;
		case 4: 
				CHS0 = 0;
				CHS1 = 0;
				CHS2 = 1;
				break;


		default: 
				return 0;
	}

	__delay_us(50);		
	
	return adc_read();

}

void init_adc(void)
{
	//set ports
	PORTA = 0;
	TRISA = 0b00111111;		//all inputs

	//set control registers
	ADCON0 = 0b10100001;									// Fosc/32, Channel 4, ADC on
	ADCON1 = 0b00000010;									// Left justified, PortE Digital, PortA Analogue

	__delay_us(50);											// Delay for ADC aquisition
}

//get an adc result from the selected channel
int adc_read(void)
{
	volatile int adc_value;

	ADRESH	=	0; 	 										// Reset the ADRESH value register 
	ADRESL	=	0; 	 										// Reset the ADRESL value register 

	GO = 1;				  									// Staring the ADC process 					
  	while(GO) continue;			   							// Wait for conversion complete 			
//	asm("nop");
//	asm("nop");
//	asm("nop");		
		

	adc_value = (ADRESH * 4) + (ADRESL / 64);				// Combine ADRESH and ADRESL to make 10-bit number. Shift ADRESH left 2 places and shidt ADRESL right 6 places

  	return (adc_value);     								// Return the value of the ADC process
}

#endif