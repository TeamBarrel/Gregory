#define BUILD // Comment this line out to omit from build
#if defined(BUILD)

#include "ir.h"
#include "xtal.h"

/* Converts ADC value into cm 											   */
/* Uses linear approximation between chosen points of known ADC/cm values  */
/* -2 to -4 offset correction for needed for +/-5% accuracy 			   */
/* Displays 999 for long error reading and 0 for short error reading 	   */
int convert(int adc_value)
{
	if(adc_value < 82)
		return 999;
	else if(adc_value < 133)
		return (100 + (150-100)*(133 - adc_value)/(133 - 82) - 3);
	else if(adc_value < 184)
		return (70 + (100-70)*(184 - adc_value)/(184 - 133) - 3);
	else if(adc_value < 256)
		return (50 + (70-50)*(256 - adc_value)/(256 - 184) - 4);
	else if(adc_value < 317)
		return (40 + (50-40)*(317 - adc_value)/(317 - 256) - 3);
	else if(adc_value < 410)
		return (30 + (40-30)*(410 - adc_value)/(410 - 317) - 2);
	else if(adc_value < 522)
		return (20 + (30-20)*(522 - adc_value)/(522 - 410) - 2);
	else return 0;
}

// Returns true if the IR sensor detects something less than 100cm away
bit findWall()
{
	if(readIR() > 100)
		return 0;
	else
		return 1;
}

/* Reads analogue voltage from IR sensor   */
/* Performs ADC conversion, and then to cm */
/* And displays on LCD screen              */
int readIR()
{
	int cm = convert(adc_read_channel(0));															// Convert the ADC value into a distance (in cm)
	return cm;
}

void rotateIR(char steps, char direction)
{
	PORTC |= 0b00000011; // Set CPLD to SM mode
	SSPBUF = direction; // Clockwise half-steps
	__delay_ms(200);

	for (char stepNum = 1; stepNum <= steps; ++stepNum)										// For the amount of steps desired
	{
		PORTC |= 0b00000100;																/* Pulse SM_STEP */
		PORTC &= 0b11111011;	
		__delay_ms(20);
	}

	SSPBUF = 0b00000000; // Clear SSPBUF
	__delay_ms(200);
	
}

#endif