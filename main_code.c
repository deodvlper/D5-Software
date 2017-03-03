#include <avr/io.h>
#include <stdlib.h>
#include <stdio.h>
#include "lcd.h"
#define F_CPU 12000000UL //added to change CPU speed to 12MHz UL=unsigned long.
#include <avr/delay.h> //added for sleep.

//FUNCTION LIST
void init_adc();
uint16_t read_adc();

int main()
{
	init_lcd();
	set_orientation(North);

	//testing write check size doubling
	//display_string("Lorem ipsum dolor sit amet, consectetur adipiscing elit.");

	//CAN MAKE THIS AS A SINGLE FUNCTION --------
	//check drawing rectangle
	rectangle rect1 = {.left=0, .right=display.width , .top=0, .bottom=17};
	fill_rectangle(rect1,WHITE);

	//testing change position, background, foreground, and write font
	//tested and working as expected
	change_position(1,2);
	change_foreground(BLACK);
	change_background(WHITE);
	display_string("Team H");

	//trying to make a square outside as a frame -- dependent on the header based on the height only. width no.
	rectangle rect2 = {.left=5, .right=display.width-5 , .top=rect1.bottom+13, .bottom=display.height-5}; //23 because 17+5
	fill_rectangle(rect2,WHITE);

	//the width and height depended on the outside frame.
	rectangle rect3= {.left=rect2.left+2, .right=rect2.right-2, .top=rect2.top+2, .bottom=rect2.bottom-2};
	fill_rectangle(rect3, BLACK);

	//writing status
	change_position(13,23); //8 because move from the left a bit, start at the same position as the square.
	change_foreground(WHITE);
	change_background(BLACK);
	display_string("Status");

	//END HERE ----  Make this one as the interface initialisation.

	//testing write down in the table
	init_table(rect3.left, rect3.top);
	for(int i=0; i<16; i++)
	{
		update_table(i,1,"Hello");
	}

	for(int i=0; i<16; i++)
	{
		update_table(i,0,"Power  :");
	}
	update_table(0,0,"HELLO");

	//VARIABLES
	uint16_t read_data;
	double voltage;
	char buffer[20];
	char buffer2[20]; //added to do conversion float using dtostrf() --INCLUDED IN STDLIB.H

	//INITIALISATION. INIT ADC SHOULD BE OUTSIDE.
	init_adc();

	//voltage value initialization.
	voltage = 11.23;

	while(1)
	{
		//ADC TESTING PLAYGROUND
		read_data = read_adc();
		//voltage = read_data/1024*3.3;
		itoa(read_data, buffer2,10); //convert read_data at buffer, decimal interpretation
		//update_table(1,1,"     "); //clearing screen for now. But can add function.
		sprintf(buffer, "%6s",buffer2);
		update_table(1,1,buffer); //updating data using adc values.

		voltage = (read_data/1023.0)*3.3; //SHOULD HAVE A 1023.0 AS THE DIVISOR otherwise use integer division and output =0 or 1.
		dtostrf(voltage,3,4,buffer2);
		//sprintf(buffer,"%6s",buffer2); //using string and 10 characters wide
		update_table(0,1,buffer2); //put it in the screen

		_delay_ms(140);

	}
	return 0;
}

void init_adc()
{
	//testing ADC in the port a, pin 0
	DDRA &= ~_BV(PINA0); //setting pin 0 as the ADC input.
	ADMUX=0; //select channel 0.
	ADCSRA |= _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1); //ADC enable. Pre-scaler f_cpu/64, not free running.
}

uint16_t read_adc()
{
	//reading ADC
	ADCSRA |= _BV(ADSC); //start conversion
	//CURRENTLY CANNOT READ
	while(ADCSRA & _BV(ADSC)){}; //wait until the ADC conversion is done.
	return ADC; //return the ADC data after ready.

}
