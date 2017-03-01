#include <avr/io.h>
#include <avr/iom644p.h> //Remove the error unresolved constant.
#include <avr/interrupt.h>
#include <avr/delay.h> //added for sleep.
#include <stdlib.h>
#include <stdio.h>
#include "lcd.h"
#define F_CPU 12000000UL //added to change CPU speed to 12MHz UL=unsigned long.

//DEFINITION: ADC
#define BBVOLTAGE PINA0
#define BBCURRENT PINA1
#define WCURRENT  PINA2	//wind current
#define SCURRENT  PINA3 //solar current

//DEFINITION: DATA-PRITING -- dtostrf -- change PREC to change precision.
#define WD 3
#define PREC 2

//DEFINITION: GLOBAL TIMER
#define OCR1A_VAL 46874 //4 overflow/second. 4 -> 46874. 5 -> 37499
#define PERIOD_OV 1/OCR1A_VAL

//DEFINITION: DIGITAL INPUT
#define DDRIN DDRA
#define PORTIN PORTA
#define PININ PINA
#define CLOAD1 PA4 //call load
#define CLOAD2 PA5
#define CLOAD3 PA6

//DEFINITION: DIGITAL OUTPUT
#define DDROUT DDRD
#define PORTOUT PORTD
#define PINOUT PIND
#define CBATT PD0 //Charge battery pin
#define DBATT PD1 //Discharge battery pin
#define SLOAD1 PD2 //switch load
#define SLOAD2 PD3
#define SLOAD3 PD4

//FUNCTION LIST
void init_usr_intfc();
void init_adc();
void init_adc_timer();
void init_global_timer();
void init_pwm();
void init_digital();
void set_pwm_vout(double vin);
uint8_t get_digital(uint8_t pin);
void set_digital(uint8_t pin, uint8_t val);
double get_time();
uint16_t read_adc(uint8_t channelNum);
void update_avg(const uint16_t* voltage_read, const uint16_t* current_read, uint64_t* sample, double* instan_power, double* avg_power);
void printNumber(double* value, char* dataToStrBuff, char*sprintfBuff, uint8_t row, uint8_t col);

//GLOBAL VARIABLES
volatile uint16_t counter=0; //used double for testing. Change to uint16_t when no need display. --> 20/2/17 changed to uint16_t
volatile uint16_t bb_volt_data=0;
volatile uint16_t bb_curr_data=0;
volatile uint16_t new_data=0; //checking if it's a new data

//INTERRUPT SERVICE ROUTINE (ISR)
ISR(TIMER1_COMPA_vect)
{
	/*Adding the counter variable*/
	counter++;
}

ISR(TIMER0_COMPA_vect)
{
	/* Reading ADC when compare match  */
	bb_volt_data=read_adc(BBVOLTAGE);
	bb_curr_data=read_adc(BBCURRENT);
	new_data=1;
}


int main()
{
	//LCD INITIALIZATION
	init_lcd();
	set_orientation(North);
	init_usr_intfc();

	//VARIABLES
	char dataToStrBuff[20]; //data -> string buffer
	char sprintfBuff[20]; // data<string> -> sprintf buffer. Might remove if use dtostrf for all.

	uint64_t sample=0; //sample number
	double instan_power=0; //Have it here, so no need power calculation at total energy usage
	double avg_power=0;
	uint16_t bb_v_sample;
	uint16_t bb_c_sample;
	uint8_t updated=0;

	//INITIALIZATION
	init_adc();
	init_adc_timer();
	init_global_timer();
	init_pwm();
	init_digital();
	sei(); //enable interrupt


//	CURRENT TASK:
//		-SETUP TIMER -- DONE
//		-SETUP ISR -- DONE
//		-CHECK ENABLING AT THE START -- DONE
//		-CHECK IF TIMER WORKS. -- by printing --DONE
//		-ADD ADC CODE TO IT, SO THAT IT FEEDS FROM THEM --done
//		-ADD AVERAGING CODE IN THE MAIN	--done
//		-ADD CHECKING MECHANISM IF THE DATA HAS BEEN READ BY THE MAIN LOOP --done

	//TESTING VARIABLE
	update_table(0,0,"voltage:");
	update_table(1,0, "current:");
	update_table(2,0,"avg pwr:");
	double current=0;
	double voltage=0;

	while(1)
	{
		if(new_data)
		{
			//TAKE DATA IN.
			cli();//disable global interrupt -- prevent unatomic operation
			bb_v_sample=bb_volt_data;
			bb_c_sample=bb_curr_data;
			new_data=0;
			sei();//enable global interrupt

			//AVERAGING + INTEGRATNG
			sample++;
			update_avg(&bb_v_sample, &bb_c_sample, &sample, &instan_power, &avg_power);
			//add integrating code.
		}

		current=(double)((bb_c_sample/1023.0)*6.6-3.3);
		voltage=(double)((bb_v_sample/1023.0)*6.6-3.3);

		printNumber(&voltage, dataToStrBuff, sprintfBuff, 0,1);
		printNumber(&current, dataToStrBuff, sprintfBuff, 1,1);
		printNumber(&avg_power, dataToStrBuff, sprintfBuff, 2,1);

//		if( (counter%2==0) & !updated)
//		{
//			//UPDATING PER 1/2 SECOND
//			printNumber(&avg_power, dataToStrBuff, sprintfBuff, 0,1);
//			printNumber(&current, dataToStrBuff, sprintfBuff, 1,1);
//			updated=1;
//		}
//		else if(counter%2!=0)
//		{
//			//PREVENTING TO BE UPDATED SO MANY TIMES IN 1/2 A SECOND.
//			updated=0;
//		}
	}


	return 0;
}


void init_usr_intfc()
{
	//TEAM H title bar
	rectangle top_bar = {.left=0, .right=display.width, .top=0, .bottom=17};
	fill_rectangle(top_bar, WHITE);
	change_position(1,2);
	change_foreground(BLACK);
	change_background(WHITE);
	display_string("Team H");

	//Status frame
	rectangle outter_rect = {.left=5, .right=display.width-5, .top=top_bar.bottom+13, .bottom=display.height-5};
	fill_rectangle(outter_rect,WHITE);
	rectangle inner_rect = {.left=outter_rect.left+2, .right=outter_rect.right-2, .top=outter_rect.top+2, .bottom=outter_rect.bottom-2};
	fill_rectangle(inner_rect, BLACK);

	//Write "Status"
	change_position(13,23);
	change_foreground(WHITE);
	change_background(BLACK);
	display_string("Status");

	//Setting up tabling grid starting point
	init_table(inner_rect.left, inner_rect.top);


}

void init_adc()
{
	/* Initializing ADC Pins */
	DDRA &= ~( _BV(BBVOLTAGE) | _BV(BBCURRENT) | _BV(WCURRENT) | _BV(SCURRENT) ); //setting pins as input at A
	ADCSRA |= _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1); //ADC enable. Pre-scaler f_cpu/64, not free running.
}

void init_adc_timer()
{
	/* Timer for ADC sampling at 625 Hz*/
	TCCR0A |= _BV(WGM01); //CTC MODE
	TCCR0B |= _BV(CS02); //256 PRESCALER
	OCR0A = 74; //COMPARE VALUE
	TIMSK0 |= _BV(OCIE0A); //ENABLING INTERRUPT COMAPRE A
}

void init_global_timer()
{
	/*Use timer 1 (16-bit timer) as the global timer -- including interrupt*/

	//Configuring Timer
	//No OC1A/B connections on port. Normal operation COM1A/B1:0=0
	//CTC Mode WGM13:0=4
	TCCR1A = 0; //setting to zeroes.
	TCCR1B |= _BV(WGM12); //setting CTC mode
	TCCR1B |= _BV(CS11) | _BV(CS10); //64 prescaler
	OCR1A = OCR1A_VAL;

	//Enable interrupts
	TIMSK1 |= _BV(OCIE1A); //local interrupt -- just ignore the error. Can built fine. Maybe Eclipse glitch. else can #include <avr/iom644p.h>
}

void init_pwm()
{
	/*Initializing PWM using timer 2 (8-bit timer)*/
	//Plan: Use Fast PWM, non-inverting mode. Higher compare register -> higher duty cycle
	//Configure OCR2A to change the duty cycle. 0->255.

	//Data direction of OC2A
	DDRD |= _BV(PD7); //Output at PD7 of PWM

	//Configuring Timer
	TCCR2A |= _BV(COM2A1) | _BV(WGM21) | _BV(WGM20); //Non-inverting mode output compare A, Fast PWM Mode with top=0xFF (mode 3). -- Ignore error. Still built, I think eclipse glitch.
	TCCR2B |= _BV(CS20); //No pre-scaling. Output frequency = 12MHz/256 = 46.875KHz

	//Initially 0V output
	OCR2A = 0;
}

void init_digital()
{
	/* Initializing digital input/output */
	DDRIN &= ~(_BV(CLOAD1) | _BV(CLOAD2) | _BV(CLOAD3)); //input setting
	PORTIN &= ~(_BV(CLOAD1) | _BV(CLOAD2) | _BV(CLOAD3)); //HI-Z input. No pull up because no switch
	DDROUT |= _BV(CBATT) | _BV(DBATT) | _BV(SLOAD1) | _BV(SLOAD2) | _BV(SLOAD3); //output setting
	PINOUT = 0; //initially all output==0;
}

void set_pwm_vout(double vin)
{
	/*Convert 0->10v (amplified out) to 0->255 */
	OCR2A = (int)((vin/10.0)*255); //added 0.5 -> when x.y -> 0<=y<=4 round down, 0<5 = round up.
}

uint8_t get_digital(uint8_t pin)
{
	/* Getting data from selected pins */
	return PININ&_BV(pin);
}

void set_digital(uint8_t pin, uint8_t val)
{
	/* Giving digital output -- opposite of the normal logic because this is the control signal for transistor */
	if (val)
	{
		PORTOUT &= ~_BV(pin);
	}
	else
	{
		PORTOUT |= _BV(pin);
	}
}

double get_time()
{
	/*Getting timer time in terms of second */
	return counter*PERIOD_OV;
}

uint16_t read_adc(uint8_t channelNum)
{
	/* Function to read ADC Values */
	ADMUX = channelNum; //change channel
	ADCSRA |= _BV(ADSC); //start conversion
	while(ADCSRA & _BV(ADSC)){}; //wait until the ADC conversion is done.
	return ADC; //return the ADC data after ready.
}

void update_avg(const uint16_t* voltage_read, const uint16_t* current_read, uint64_t* sample, double* instan_power, double* avg_power)
{
	//THIS FUNCTION SEEMS NOT TOO GOOD.
	*sample+=1;
	//*instan_power = ((*voltage_read/1023.0)*8.0-4)*100 * ((*current_read/1023)*20.0-10);
	*instan_power = ((*voltage_read/1023.0)*6.6-3.3) * ((*current_read/1023.0)*6.6-3.3); //Note wrong offset before this.
	*avg_power = *avg_power + (*instan_power-*avg_power) / *sample;
}

void printNumber(double* value, char* dataToStrBuff, char* sprintfBuff, uint8_t row, uint8_t col)
{
	/*Updating double/integer to certain precision to screen in table mode */
	dtostrf(*value,WD,PREC, dataToStrBuff);
	sprintf(sprintfBuff, "%6s", dataToStrBuff);
	update_table(row, col, sprintfBuff);
}
