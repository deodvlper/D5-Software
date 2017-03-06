/* TO DO:
 * 	Testing the averaging code after changing from = to += of line 299
 * 	Testing the total energy function. Does it work properly.
 * 	Add control for the first profile.
 */
 
//From Fransiskus's branch. 

 //ASSUMING EVERYTHING IS IN AC RMS CURRENT

 
// Ricki's branch comment
 
#include "helloWorld.h"

//GLOBAL VARIABLES
volatile uint8_t counter = 0; 		//used double for testing. Change to uint16_t when no need display. --> 20/2/17 changed to uint16_t
									//counter is the real time in seconds
volatile uint8_t temp = 0;			//used in the interrupt as a temporary variable for the 'counter' variable
volatile uint16_t bb_volt_data = 0; //bus bar voltage
volatile uint16_t bb_curr_data = 0; //bus bar current
volatile uint16_t wt_curr_data = 0; //wind turbine current
volatile uint16_t pv_curr_data = 0;	//solar panel current
volatile uint16_t new_data = 0; 	//checking if it's a new data

volatile char dataToStrBuff[20];    //data (double) -> string buffer (array of chars), used in dtostrf	
volatile char sprintfBuff[20];      //data<string> -> sprintf buffer. Formats array of chars into suitable format for display,
								    //this is what is displayed on the LCD

ISR(TIMER0_COMPA_vect)
{
	/* Reading ADC when compare match  */
	bb_volt_data = read_adc(BBVOLTAGE);
	bb_curr_data = read_adc(BBCURRENT);
	wt_curr_data = read_adc(WTCURRENT);
	pv_curr_data = read_adc(PVCURRENT);
	new_data=1;
	
	temp += 1;
	if (temp == 125)		//when temp reaches 20% of the count (0.2s)
		{
			counter +=1;	//global clock increments every 0.2s
			temp = 0;
		}
}

int main()
{
	//LCD INITIALIZATION
	init_lcd();				//Premade function, configures the ports
	set_orientation(North);	//Premade funtion, Sets in portrait mode
	init_usr_intfc();		//Created function, draws the main theme, sets up table
	
	//VARIABLES
	//char dataToStrBuff[20]; //data (double) -> string buffer (array of chars), used in dtostrf
	//char sprintfBuff[20];   //data<string> -> sprintf buffer. Formats array of chars into suitable format for display,
							  //this is what is displayed on the LCD

	uint64_t sample = 0; 		//sample count
	uint16_t bb_v_sample = 0;	//updates on each sample
	uint16_t bb_c_sample = 0;	//updates on each sample
	uint16_t wt_c_sample = 0;	//updates on each sample
	uint16_t pv_c_sample = 0;	//updates on each sample
	double total_energy = 0;	
	double avg_power = 0;	
	uint8_t updated = 0;		//acts as a boolean variable, used for updating LCD

	uint8_t load1_r = 0; 		//'_r' = request, '_s' = set.
	uint8_t load2_r = 0;		//all of these act as boolean variables
	uint8_t load3_r = 0;
	
	uint8_t load1_s = 0; 	
	uint8_t load2_s = 0;		
	uint8_t load3_s = 0;
	
	uint8_t battery_c = 0; 		//'_c' = charge, '_d' = discharge
	uint8_t battery_d = 0;		//
	
	double i_mains = 0;
	
	//INITIALIZATION
	init_adc();					//Created function, enables ADC pins 
	init_adc_timer();			//Created function, sets up 
	init_pwm();					//sets up the registers, for the voltage output pin
	init_digital();				//sets up the digital inputs on port A, outputs on port D
 						
	set_digital(SLOAD1, 0);
	set_digital(SLOAD2, 0);
	set_digital(SLOAD3, 0);	
	set_digital(CBATT, 0);
	set_digital(DBATT, 0);
	set_pwm_vout(0);
	
	//TESTING VARIABLE
	double voltage = 0;
	double current = 0;
	double wt_current = 0;
	double pv_current = 0;
	double test;

	sei();					//enable interrupt
	
	while(1)
	{
		if(new_data)
		{
			//TAKE DATA IN.
			cli();						//disable global interrupt -- prevent unatomic operation
			bb_v_sample = bb_volt_data;
			bb_c_sample = bb_curr_data;
			wt_c_sample = wt_curr_data;
			pv_c_sample = pv_curr_data;
			new_data = 0;
			sei();						//enable global interrupt

			//INTEGRATING AND AVERAGING
			update_energy(&bb_v_sample, &bb_c_sample, &sample, &total_energy);
			update_avg(&total_energy, &sample, &avg_power);
		}

		//DECISION SATEMENT FOR THE FIRST ONE
		//taking in the load request values
		load1_r = (get_digital(CLOAD1)) ? 1 : 0;
		load2_r = (get_digital(CLOAD2)) ? 1 : 0;
		load3_r = (get_digital(CLOAD3)) ? 1 : 0;
		
		battery_c = ((load1_r*I1+load2_r*I2+load3_r*I3) <= 3) ? 1 : 0;

		set_digital(SLOAD1,load1_s);
		set_digital(SLOAD2,load2_s);
		set_digital(SLOAD3,load3_s);
		set_digital(CBATT, battery_c);
		set_digital(DBATT, battery_d);

		//DIRECT SCREEN UPDATE - CHANGE LATER
		//Finding voltage and current
		voltage = (double)((bb_v_sample/1023.0)*6.6-3.3);
		current = (double)((bb_c_sample/1023.0)*6.6-3.3);
		wt_current = (double)((wt_c_sample/1023.0)*5);
		pv_current = (double)((pv_c_sample/1023.0)*5);
		
		//Updating display
		update_values(voltage, current, load1_r, load2_r, load3_r, load1_s, load2_s, load3_s, battery_c, battery_d, (i_mains*10), wt_current, pv_current);			//Update values

/*
		test = (double)counter;
		printNumber(&test, dataToStrBuff, sprintfBuff, 4,1);

		
		if( (counter%2==0) & !updated)
		{
			//UPDATING PER 1/2 SECOND


			updated=1;
		}
		else if(counter%2!=0)
		{
			//PREVENTING TO BE UPDATED SO MANY TIMES IN 1/2 A SECOND.
			updated=0;
		}
*/
		//
	}

	return 0;
}

void init_usr_intfc()	
{
	/*draws the main theme, sets up table*/
    //Clears screen, makes black, ready for data
    rectangle clear_screen = {.left=0, .right=(display.width-1), .top=0, .bottom=display.height};
    fill_rectangle(clear_screen, BLACK);

    //TEAM H title bar
    rectangle top_bar = {.left=0, .right=(display.width-1), .top=0, .bottom=19};
    fill_rectangle(top_bar, WHITE);

    change_position(13,2);
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
	
	//Writes the text
	update_table(0,0, "C1:");
	update_table(1,0, "C2:");
	update_table(2,0, "C3:");
	
	update_table(0,2, "S1:");
	update_table(1,2, "S2:");
	update_table(2,2, "S3:");
	
	update_table(3,0, "B_Status:");
	update_table(4,0, "B_Level:");
	update_table(5,0, "%_Mains:");
	update_table(6,0, "I_Mains:");
	update_table(7,0, "I_Wind:");
	update_table(8,0, "I_Solar:");
	update_table(9,0, "BB_V");
	update_table(10,0, "BB_C");
	
	update_table(4,4, "J");
	update_table(5,4, "%");
	update_table(6,4, "A");
	update_table(7,4, "A");
	update_table(8,4, "A");
	update_table(9,4, "V");
	update_table(10,4, "A");
}

void init_adc()			 
{
	/* Initializing ADC Pins */
	DDRA &= ~( _BV(BBVOLTAGE) | _BV(BBCURRENT) | _BV(WTCURRENT) | _BV(PVCURRENT) ); //Setting 4 pins on port A as inputs
	ADCSRA |= _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1); 								  //ADC enable. Pre-scaler f_cpu/64, not free running.
}

void init_adc_timer()
{
	/* Timer for ADC sampling at 625 Hz*/
	TCCR0A |= _BV(WGM01); 			//CTC MODE
	TCCR0B |= _BV(CS02); 			//256 PRESCALER
	OCR0A = 74; 					//COMPARE VALUE
	TIMSK0 |= _BV(OCIE0A); 			//ENABLING INTERRUPT COMAPRE A
}

void init_pwm()				
{
	/*Sets up the PWM timer 2 registers (8-bit timer), for the voltage output pin*/
	//Plan: Use Fast PWM, non-inverting mode. Higher compare register -> higher duty cycle
	//Configure OCR2A to change the duty cycle. 0->255.

	//Data direction of OC2A
	DDRD |= _BV(PD7); //Output at PD7 of PWM

	//Configuring Timer
	TCCR2A |= _BV(COM2A1) | _BV(WGM21) | _BV(WGM20); //Non-inverting mode output compare A, Fast PWM Mode with top=0xFF (mode 3). -- Ignore error. Still built, I think eclipse glitch.
	TCCR2B |= _BV(CS20); 							 //No pre-scaling. Output frequency = 12MHz/256 = 46.875KHz

	//Initially 0V output
	OCR2A = 0;
}

void init_digital()			
{
	/*sets up the digital inputs on port A, outputs on port D*/
	DDRIN &= ~(_BV(CLOAD1) | _BV(CLOAD2) | _BV(CLOAD3)); 						 //setting inputs
	PORTIN &= ~(_BV(CLOAD1) | _BV(CLOAD2) | _BV(CLOAD3)); 						 //HI-Z input. No pull up because no switch
	DDROUT |= _BV(CBATT) | _BV(DBATT) | _BV(SLOAD1) | _BV(SLOAD2) | _BV(SLOAD3); //setting outputs
	PINOUT = 0; 																 //initially all output==0;
}

void set_pwm_vout(double vin)
{
	/*Convert 0->10v (amplified out) to 0->255 */
	OCR2A = (int)(((vin/10.0)*255)+0.5); //added 0.5, for x.y when 0<=y<=4 round down, 0<5 = round up.
										 //converting to int truncates the decimal, +0.5 acts as rounding
										 //sets the compare value as the voltage required 
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
	return (counter/5);
}

uint16_t read_adc(uint8_t channelNum)
{
	/* Function to read ADC Values */
	ADMUX = channelNum; 			//change channel
	ADCSRA |= _BV(ADSC); 			//start conversion
	while(ADCSRA & _BV(ADSC)){}; 	//wait until the ADC conversion is done.
	return ADC; 					//return the ADC data after ready.
}

void update_energy(const uint16_t* voltage_read, const uint16_t* current_read, uint64_t* sample, double* total_energy)
{
	/* Updating the total energy*/
	*sample+=1;
	*total_energy += ((*voltage_read/1023.0)*6.6-3.3) * ((*current_read/1023.0)*6.6-3.3) * 0.0016; //0.0016 is sampling period of 625Hz.
	//*total_energy += ((*voltage_read/1023.0)*6.6-3.3)*((*voltage_read/1023.0)*6.6-3.3);
}

void update_avg(const double* total_energy,const uint64_t* sample,double* avg_power)
{
	/* UPDATE AVG REV 2*/ 
	//NOW GET AVERAGE FROM TOTAL VALUE.
	*avg_power = *total_energy / (*sample * 0.0016); //divide by the total time
}

void printNumber(double* value, char* dataToStrBuff, char* sprintfBuff, uint8_t row, uint8_t col)
{
	/*Updating double/integer to certain precision to screen in table mode */
	dtostrf(*value,WD,PREC, dataToStrBuff);			//turns doubles into an array of chars
	sprintf(sprintfBuff, "%7s", dataToStrBuff);		//formats array of chars into suitable format for display
	update_table(row, col, sprintfBuff);
}

void update_values(double bb_v, double bb_c, uint8_t load1_r, uint8_t load2_r, uint8_t load3_r, uint8_t load1_s, uint8_t load2_s, uint8_t load3_s, uint8_t battery_c, uint8_t battery_d, double i_mains, double wt_current, double pv_current)
{
	printNumber(&bb_v, dataToStrBuff, sprintfBuff, 9,2);			//Update voltage value
	printNumber(&bb_c, dataToStrBuff, sprintfBuff, 10,2);			//Update current value
	
	printNumber(&i_mains, dataToStrBuff, sprintfBuff, 5,2);			//Update mains current value
	printNumber(&wt_current, dataToStrBuff, sprintfBuff, 7,2);
	printNumber(&pv_current, dataToStrBuff, sprintfBuff, 8,2);
	
	(load1_r) ? update_table(0,1, "Yes") : update_table(0,1, "No ");	//Update load 1 request
	(load2_r) ? update_table(1,1, "Yes") : update_table(1,1, "No ");	//Update load 2 request
	(load3_r) ? update_table(2,1, "Yes") : update_table(2,1, "No ");	//Update load 3 request
	
	(load1_s) ? update_table(0,3, "Yes") : update_table(0,3, "No ");	//Update load 1 request
	(load2_s) ? update_table(1,3, "Yes") : update_table(1,3, "No ");	//Update load 2 request
	(load3_s) ? update_table(2,3, "Yes") : update_table(2,3, "No ");	//Update load 3 request
	
	if ((battery_c == 0) && (battery_d == 0))
		update_table(3,2, " Idle");
	else if (battery_c == 1)
		update_table(3,2, " Charge");
	else 
		update_table(3,2, " Discharge");
	
	
}
