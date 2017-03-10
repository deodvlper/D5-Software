 //ASSUMING EVERYTHING IS IN AC RMS CURRENT
 //CHECK VOLTAGE.

#include "helloWorld.h"

//GLOBAL VARIABLES
volatile uint32_t counter = 0;  //32 bits counter handling 1ms interval
volatile char dataToStrBuff[20];    //data (double) -> string buffer (array of chars), used in dtostrf
volatile char sprintfBuff[20];      //data<string> -> sprintf buffer. Formats array of chars into suitable format for display,
								    //this is what is displayed on the LCD

volatile uint8_t battery_c 			 = 0; 			//'_c' = charge, '_d' = discharge
volatile double charge_start_time    = 0;
volatile double charge_end_time      = 0;

volatile uint8_t battery_d 			 = 0;											
volatile double discharge_start_time = 0;
volatile double discharge_end_time   = 0;

volatile double battery_capacity	 = 0;
			
//INTERRUPT SERIVE ROUTINE (ISR)
ISR(TIMER1_COMPA_vect)
{
  /* Adding global counter */
  counter += 1;
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

	uint64_t sample		 = 0; 	//sample count
	
	//uint16_t bb_v_sample = 0;	//updates on each sample
	//uint16_t bb_c_sample = 0;	//updates on each sample
	//uint16_t wt_c_sample = 0;	//updates on each sample
	//uint16_t pv_c_sample = 0;	//updates on each sample
	
	double total_energy  = 0;	
	double avg_power     = 0;	
	uint8_t updated      = 0;	//acts as a boolean variable, used for updating LCD

	uint8_t load1_r = 0; 		//'_r' = request, '_s' = set.
	uint8_t load2_r = 0;		//all of these act as boolean variables
	uint8_t load3_r = 0;
	
	uint8_t load1_s = 0; 	
	uint8_t load2_s = 0;		
	uint8_t load3_s = 0;
	
	double I_mains = 0;

	uint8_t lcd_count = 0;		//acts as a boolean variable, used for updating LCD

	double i_total_need = 0;
	double i_renew_total = 0;
	
	double voltage   = 0;
	double current   = 0;
	uint16_t I_wind  = 0;
	uint16_t I_solar = 0;
	double test;
	
	uint16_t I_required  = 0;
	uint16_t I_renewable = 0;

	//INITIALIZATION
	init_lcd();					//Premade function, configures the ports
	set_orientation(North);		//Premade funtion, Sets in portrait mode
	init_usr_intfc();			//Created function, draws the main theme, sets up table

	init_adc();					//Created function, enables ADC pins
	init_global_timer();
	init_pwm();					//sets up the registers, for the voltage output pin
	init_digital();				//sets up the digital inputs on port A, outputs on port D
 					
	sei();						//enable interrupt
	
	init_loads_pwm(); 			//sets load switches and PWM output to zero

	while(1)
	{

		/* 1) FUNCTION 1 Check load calls, turn off unwanted loads, calculate required current, store in variable I_required */
		
		load1_r = (get_digital(CLOAD1)) ? 1 : 0;	//store load call in local variable
		if (load1_r == 0)
			{
				load1_s = load1_r;
				set_digital(SLOAD1,load1_s);		//if load call is 0, set switch to 0
			}
			
		load2_r = (get_digital(CLOAD2)) ? 1 : 0;	//store load call in local variable
		if (load2_r == 0)
			{
				load2_s = load2_r;	
				set_digital(SLOAD1,load2_s);		//if load call is 0, set switch to 0
			}
			
		load3_r = (get_digital(CLOAD3)) ? 1 : 0;	//store load call in local variable
		if (load3_r == 0)
			{
				load3_s = load3_r;
				set_digital(SLOAD1,load3_s);		//if load call is 0, set switch to 0
			}
			
		I_required = (load1_r * I1) + (load2_r * I2) + (load3_r * I3);		//finding required current 
		
		/* 2) FUNCTION 2 Use ADC single read of WT and PV to find current from renewables, store in variable */
		
		I_wind  = read_adc(WTCURRENT);	//obtain ADC value from 0 to 1023
		I_wind  = (I_wind/1023.0) * 5;	//turns ADC value into the respective current
		
		I_solar = read_adc(PVCURRENT);	//obtain ADC value from 0 to 1023
		I_solar = (I_solar/1023.0) * 5;	//turns ADC value into the respective current
	
		I_renewable = I_wind + I_solar; 
		
		/* 3) DECISION MAKING BLOCKS */
		
		/* Enough current from renewables?*/
		if (I_renewable > I_required)
			{	
				/* Yes */
				if (battery_d == 1)				//checks if currently discharging
					{
						battery_d = 0;
						battery_control(0,1);	//stops discharging if it is discharging
					}
					
				/* 1A surplus current from renewables */
				if ((I_renewable - I_required) > 1)	//checks if there is a 1A surplus (enough to charge battery)
					{
						if (battery_c == 1)			//checks if battery is already charging
							{
								//connect up loads
								//control function
							}
						else 
							{
								//charge battery (function)
								//connect up loads
								//control function
							}
					}
				
				/* <1A surplus current from renewables */
				/* Battery capacity check */
				else if (battery_capacity > 8)	//NEED TO CHANGE, 8 IS NOT 8 ACTUAL MINUTES
					{
						if (battery_c == 1)					//if battery is already charging
							{
								//stop charging (function)	//battery capacity is enough
							}
						//connect up loads
						//control function?
					}
				else /* Battery capacity is low, can charge*/
					{
						if (battery_c == 1)					//if battery is already charging
							{
								//connect up loads
								//control function?
							}
						else 
							{
								//find out how much more current is needed from the mains to charge battery
								//start charging (function)
								//connect up loads
								//control function
							}
					}
			}
		
		/* When renewables is not enough to supply load*/
		else
			{
				if ((I_required - I_renewable) < 1)		//if the current left to supply is less than 1A
					/* Yes, use mains to supply deficit */
					{
						if (battery_d == 1)				
							{
								//stop discharging (function)
							}
						if (battery_capacity > 8)		//NEED TO CHANGE, 8 IS NOT 8 ACTUAL MINUTES
							{
								//find mains current to supply remaining current
								//set PWM signal
								//connect up loads
								//control function
							}
						else 
							{
								if (battery_c != 1)
									{
										//start charging (function)
									}
								//find mains current to supply remaining current + battery charging
								//set PWM signal
								//connect up loads
								//control function
							}
					}
				else
					/* No, use mains and possible battery to supply deficit*/ 
					{
						if (battery_capacity > 3)		//NEED TO CHANGE, 3 IS NOT 3 ACTUAL MINUTES (3 is used so we have spare in case power goes down)
						/* if there is capacity, then discharge the battery */
							{
								if (battery_c == 1)
									{
										//stop charging (function)
									}
								if (battery_d != 1)
									{
										//start discharging (function)
									}
								//find remaining current to supply by mains
								//set PWM signal
								//connect up loads
								//control function
							}
						else
						/* Not enough capacity in battery, so use mains to supply deficit*/
							{
								if (battery_d == 1)
									{
										//stop discharging (function)
									}
								//find remaining current left to supply (I_required - I_renewable)
								//check if ^ + ~1.2 is less than 3A
								//if yes, then charge battery, get mains to supply
								//if no, then just use mains to supply loads
								
								//connect up loads
								//control function
							}
					}
			}
		
		/* 4) CONNECT UP LOADS, CONTROL BLOCK */
		//PEAK FINDER

		/* 5) DISPLAYING ON THE LCD */
		//Displaying per second
		lcd_count++;

		if(lcd_count == 5) //vary this to change screen update speed.
		{
			voltage = get_v_amp();
			//printNumber(&test, dataToStrBuff, sprintfBuff, 9,2);
			current = get_c_amp();
			//printNumber(&test, dataToStrBuff, sprintfBuff, 10,2);
			update_values(voltage, current, load1_r, load2_r, load3_r, load1_s, load2_s, load3_s, battery_c, battery_d, I_mains, I_wind, I_solar, battery_capacity);			//Update values
			lcd_count = 0; //reset count for updating the screen.
		}
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

	//update_table(4,4, "J");
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
	ADCSRA |= _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1); 								    //ADC enable. Pre-scaler f_cpu/64, not free running.
}

void init_global_timer()
{
  /* Timer for global timer, working at 1ms interval*/
  TCCR1A |= 0;
  TCCR1B |= _BV(WGM12); //ctc mode.
  TCCR1B |= _BV(CS11); 	//setting prescaler to 8.
  OCR1A = 1499; 		//Max value to count every 1ms.

  //enable interrupt
  TIMSK1 |= _BV(OCIE1A); //enable its interrupt.
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

void init_loads_pwm()
{
  /* Initialising the load and pwm out */
  set_digital(SLOAD1, 0);
  set_digital(SLOAD2, 0);
  set_digital(SLOAD3, 0);
  set_digital(CBATT,  0);
  set_digital(DBATT,  0);
  set_pwm_vout(0);
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

double get_v_amp() //NOTE: Amplitude value
{
  uint16_t bb_q_sample = 0; 			 //reset value of bb var(s) every starting point.
  uint16_t bb_q_amp    = 0;
  uint32_t peak_end_time = counter + 10; //get how long does it need to wait to get the amplitude

  while(counter < peak_end_time)
  {
	//sampling bb volt or current
	bb_q_sample = abs(read_adc(BBVOLTAGE)-511); //Mid point (1024/2) - 1 = 511, finds absolute value

	//finding peak
	if(bb_q_sample > bb_q_amp)
		{
		  //if sample is more than prev sampled value replace it.
		  bb_q_amp = bb_q_amp;
		}
  }

  //return value
  return (bb_q_amp/512.0)*400; //because max input at 400V.
}

double get_c_amp() //NOTE: RMS VALUE
{
  uint16_t bb_q_sample = 0; 			 //reset value of bb var(s) every starting point.
  uint16_t bb_q_amp    = 0;
  uint32_t peak_end_time = counter + 10; //get how long does it need to wait to get the amplitude

  while(counter < peak_end_time)
  {
    //sampling bb volt or current
    bb_q_sample = abs(read_adc(BBCURRENT)-511); //Mid point (1024/2) - 1 = 511

    //finding peak
    if(bb_q_sample > bb_q_amp)
		{
		  //if sample is more than prev sampled value replace it.
		  bb_q_amp = bb_q_amp;
		}
  }

  //return value
  return (bb_q_amp/512.0)*10/sqrt(2); //because max 10v~10A amplitude rms value
}

uint16_t read_adc(uint8_t channelNum)
{
	/* Function to read ADC Values */
	ADMUX = channelNum; 			//change channel
	ADCSRA |= _BV(ADSC); 			//start conversion
	while(ADCSRA & _BV(ADSC)){}; 	//wait until the ADC conversion is done.
	return ADC; 					//return the ADC data after ready.
}

void battery_stop(uint8_t mode, const uint32_t* start_time, uint32_t* total_time)
{
  //end charging
  if(mode)
  {
    *total_time += (counter-*start_time); //add the delta time.
    set_digital(CBATT,0); //stop charging
  }
  //end discharging
  else
  {
    *total_time -= (counter-*start_time); //minus the delta time after discharging.
    set_digital(DBATT,0);//stop discharging
  }
}

void battery_start(uint8_t mode, uint32_t* start_time)
{
  //start charging
  if(mode)
  {
    set_digital(CBATT,1); //start charging
    set_digital(DBATT,0); //stop discharging
  }
  //start discharging
  else
  {
    set_digital(CBATT,0); //stop charging
    set_digital(DBATT,1); //start discharging
  }
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

void update_values(double bb_v, double bb_c, uint8_t load1_r, uint8_t load2_r, uint8_t load3_r, uint8_t load1_s, uint8_t load2_s, uint8_t load3_s, uint8_t battery_c, uint8_t battery_d, double I_mains, double I_wind, double I_solar, double battery_capacity)
{	
	printNumber(&battery_capacity, dataToStrBuff, sprintfBuff, 4,2);//Update battery capacity value
	
	uint8_t I_mains_percentage;
	I_mains_percentage = (I_mains/3) * 100;
	printNumber(&I_mains_percentage, dataToStrBuff, sprintfBuff, 5,2);//Update mains percenatge current value
	
	printNumber(&I_mains, dataToStrBuff, sprintfBuff, 6,2);			//Update mains current value
	
	printNumber(&I_wind, dataToStrBuff, sprintfBuff, 7,2);			//Update wind current value
	printNumber(&I_solar, dataToStrBuff, sprintfBuff, 8,2);			//Update solar current value
	
	printNumber(&bb_v, dataToStrBuff, sprintfBuff, 9,2);			//Update voltage value
	printNumber(&bb_c, dataToStrBuff, sprintfBuff, 10,2);			//Update current value
	
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

void battery_control(uint8_t charge_control, uint8_t discharge_control)
{
	if (charge_control == 1)
		{
			if (battery_c == 1)
				{
					set_digital(CBATT, battery_c);		//start charging the battery
					charge_start_time = get_time();		//record charging start time
				}
			if (battery_c == 0)
				{
					set_digital(CBATT, battery_c);		//stop charging the battery
					charge_end_time = get_time();		//record charging end time
					battery_capacity += (charge_end_time - charge_start_time);
				}
		}
		
	if (discharge_control == 1)
		{
			if (battery_d == 1)
				{
					set_digital(DBATT, battery_d);		//start discharging the battery
					discharge_start_time = get_time();	//record discharging start time
				}
			if (battery_d == 0)
				{
					set_digital(DBATT, battery_d);		//stop discharging the battery
					discharge_end_time = get_time();	//record discharging end time
					battery_capacity -= (discharge_end_time - discharge_start_time);
				}			
		}
}
