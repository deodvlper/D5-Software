 //ASSUMING EVERYTHING IS IN AC RMS CURRENT
 //CHECK VOLTAGE.

#include "helloWorld.h"

//GLOBAL VARIABLES
volatile uint32_t counter = 0;  	//32 bits counter handling 1ms interval
volatile char dataToStrBuff[20];    //data (double) -> string buffer (array of chars), used in dtostrf
volatile char sprintfBuff[20];      //data<string> -> sprintf buffer. Formats array of chars into suitable format for display,
								    //this is what is displayed on the LCD

volatile uint8_t battery_c 			 = 0; 			//'_c' = charge, '_d' = discharge
volatile uint32_t charge_start_time    = 0;
volatile uint32_t charge_end_time      = 0;

volatile uint8_t battery_d 			 = 0;											
volatile uint32_t discharge_start_time = 0;
volatile uint32_t discharge_end_time   = 0;

volatile uint32_t battery_capacity	 = 0;			//each increment is in milliseconds
			
//INTERRUPT SERIVE ROUTINE (ISR)
ISR(TIMER1_COMPA_vect)
{
  /* Adding global counter */
  counter += 1;					//increments every millisecond
}

int main()
{	
	//VARIABLES
	//char dataToStrBuff[20]; //data (double) -> string buffer (array of chars), used in dtostrf
	//char sprintfBuff[20];   //data<string> -> sprintf buffer. Formats array of chars into suitable format for display,
							  //this is what is displayed on the LCD

	uint64_t sample		 = 0; 	//sample count
	
	//uint16_t bb_v_sample = 0;	//updates on each sample
	//uint16_t bb_c_sample = 0;	//updates on each sample
	//uint16_t wt_c_sample = 0;	//updates on each sample
	//uint16_t pv_c_sample = 0;	//updates on each sample
	
	double temp = 0;
	
	double   total_energy = 0;	
	double   avg_power    = 0;	
	uint8_t  updated      = 0;	//acts as a boolean variable, used for updating LCD

	uint8_t  load1_r = 0; 		//'_r' = request, '_s' = set.
	uint8_t  load2_r = 0;		//all of these act as boolean variables
	uint8_t  load3_r = 0;
	
	uint8_t  load1_s = 0; 	
	uint8_t  load2_s = 0;		
	uint8_t  load3_s = 0;
	
	double   I_mains = 0;		//in terms of current (A)
	uint8_t  mains_status = 1;	//

	uint32_t  lcd_count = 0;		//acts an update counter, used for updating LCD

	//double i_total_need  = 0;
	//double i_renew_total = 0;
	
	double   voltage = 0;		//busbar voltage
	double   current = 0;		//busbar current
	double   I_wind  = 0;		//in terms of current (A)
	double   I_solar = 0;		//in terms of current (A)
	
	double I_required  = 0;	//the total current required from the loads
	double I_renewable = 0;	//the total current coming from wind and solar
	
	uint8_t  control = 0;			//used in the while loop at the beginning, checking for changes
	uint32_t battery_drain = 0;		//used in the while loop at the beginning, checks usage
	uint8_t  load_off = 0;			//used in the while loop at the beginning, remembers what load we turned off
	double   renewable_check = 0;	//used in the while loop at the beginning, stores the value to save calculating twice
	
	//INITIALIZATION
	init_lcd();					//premade function, configures the ports
	set_orientation(North);		//premade funtion, sets in portrait mode
	init_usr_intfc();			//preated function, draws the main theme, sets up table

	init_adc();					//created function, enables ADC pins
	init_global_timer();
	init_pwm();					//sets up the registers, for the voltage output pin
	init_digital();				//sets up the digital inputs on port A, outputs on port D
 					
	sei();						//enable interrupt
	
	init_loads_pwm(); 			//sets load switches and PWM output to zero

	while (1)
	{
		while (control == 1)
			{
				update_table(12,1, " While 1     ");	
				//temp = (double)counter;
				//printNumber(&temp, dataToStrBuff, sprintfBuff, 11,2);				//Update time
				
				if (load1_r != get_digital(CLOAD1) || load2_r != get_digital(CLOAD2) || load3_r != get_digital(CLOAD3))
					control = 0;		//exit control loop, as the load requests have changed
				
				battery_drain = battery_capacity - (counter - discharge_start_time);	//shows updated battery capacity, without changing battery_capacity
				if (battery_d == 1)
					{
						if ((mains_status = 1) && (battery_drain < 180000))
							control = 0;
						if ((mains_status = 0) && (battery_drain < 10))
							control = 0;
					}
				
				renewable_check = ((read_adc(WTCURRENT))/1023.0) * 5;
				if ( (I_wind  < (renewable_check - 0.09)) || (I_wind  > (renewable_check + 0.09)) )		//checks if the wind turbine current has changed
					control = 0;
		
				renewable_check = ((read_adc(PVCURRENT))/1023.0) * 5;
				if ( (I_solar  < (renewable_check - 0.09)) || (I_solar  > (renewable_check + 0.09)) )	//checks if the solar cell current has changed
					control = 0;				
					
				if ((mains_status == 1) && (I_mains != 0))	//checks to see if the mains has gone down, when we are using the mains
					{
						voltage = get_v_amp();	//obtain busbar voltage
						if (voltage < 238)		//tolerance included here (should be 240)
							control = 0;		//exit loop
					}
				if (mains_status == 0)	//checks to see if the mains is back on
					{
						I_mains = 2;
						set_pwm_vout(I_mains);
						
						if ((load1_r == 1) && (load1_s == 0)) 
							load_off = SLOAD1;
						if ((load2_r == 1) && (load2_s == 0)) 
							load_off = SLOAD2;
						if ((load3_r == 1) && (load3_s == 0))
							load_off = SLOAD3;	
						
						set_digital(load_off,1);	//connect the load that wants to be on, but we turned off due to not enough supply
						
						voltage = get_v_amp();		//obtain busbar voltage
						if (voltage > 238)			//tolerance included here (should be 240)
							control = 0;			//exit loop//if BB v is still at 240, then the mains is working
						
						set_digital(load_off,0);	//disconnect the load that want to be on (code below will turn it back on)
						I_mains = 0;
						set_pwm_vout(I_mains);						
					}
				//check when if mains is down, detect if it is back up, change mains_status and control = 0
				
				update_table(12,1, " Before LCD  ");
				//displaying per second
				if(counter > (lcd_count * 1000)) //vary this to change screen update speed.
					{
						voltage = get_v_amp();		
						current = get_c_amp();
						update_table(12,1, " LCD update 1");
						update_values(&voltage, &current, &load1_r, &load2_r, &load3_r, &load1_s, &load2_s, &load3_s, &battery_c, &battery_d, &I_mains, &I_wind, &I_solar);			//Update values
						lcd_count += 1; //count for updating the screen.
						update_table(12,1, " LCD update 2");	
					}
			}
		
		
		/* 1) TASK 1 Check load calls, turn off unwanted loads, calculate required current, store in variable I_required */
		
		load1_r = (get_digital(CLOAD1)) ? 1 : 0;	//store load call in local request variable
		load2_r = (get_digital(CLOAD2)) ? 1 : 0;	//store load call in local request variable	
		load3_r = (get_digital(CLOAD3)) ? 1 : 0;	//store load call in local request variable
			
		I_required = (load1_r * I1) + (load2_r * I2) + (load3_r * I3);		//finding required current 
		
		/* 2) TASK 2 Use ADC single read of WT and PV to find current from renewables, store in variable */
		
		I_wind  = read_adc(WTCURRENT);	//obtain ADC value from 0 to 1023
		I_wind  = (I_wind/1023.0) * 5;	//turns ADC value into the respective current
		
		I_solar = read_adc(PVCURRENT);	//obtain ADC value from 0 to 1023
		I_solar = (I_solar/1023.0) * 5;	//turns ADC value into the respective current
	
		I_renewable = I_wind + I_solar; 
		
		printNumber(&I_renewable, dataToStrBuff, sprintfBuff, 13,2);	
		printNumber(&I_required, dataToStrBuff, sprintfBuff, 14,2);	
		
		/* 3) DECISION MAKING BLOCKS */
		
		/* Enough current from renewables?*/
	  /*  if(i_renew_total>i_total_need)
			{
			  if(battery_d==1)
				  {
					battery_stop(DISCHARGING, &start_time, &total_time, &battery_c, &battery_d);
				  }

			  if( (i_renew_total-i_total_need) >= 1)
			  {
				if(battery_c==0)
					{
					  battery_start(CHARGING,&start_time, &battery_c, &battery_d);
					}
				set_digital(SLOAD1,load1_r);
				set_digital(SLOAD2,load2_r);
				set_digital(SLOAD3,load3_r);
			  }
			}
		else if(i_renew_total<i_total_need)
			{
*/
		
		if ((I_required == 0) && (I_renewable == 0))			//goes in here at startup
			{
				set_loads(&load1_r, &load2_r, &load3_r, &load1_s, &load2_s, &load3_s);	//connect up loads
				control = 1;
			}
		
		else if (I_renewable > I_required)
			{	
				/* Yes */
				I_mains = 0;					//set mains output to 0A
				set_pwm_vout(I_mains);
				
				if (battery_d == 1)				//checks if currently discharging
					{
						battery_d = 0;
						battery_control(0,1);	//stops discharging if it is discharging
												//charge_control = 0
												//discharge_control = 1
					}
					
				/* 1A surplus current from renewables */
				if ((I_renewable - I_required) > 1)	//checks if there is a 1A surplus (enough to charge battery)
					{
						if (battery_c == 1)			//checks if battery is already charging
							{
								set_loads(&load1_r, &load2_r, &load3_r, &load1_s, &load2_s, &load3_s);	//connect up loads
							}
						else 
							{
								battery_c = 1;
								battery_control(1,0);
								set_loads(&load1_r, &load2_r, &load3_r, &load1_s, &load2_s, &load3_s);	//connect up loads
							}
					}
				
				/* <1A surplus current from renewables */
				/* Battery capacity check */
				else if (battery_capacity > 480000)		//480000ms = 8 minutes
					{
						if (battery_c == 1)					//if battery is already charging
							{
								battery_c = 0;
								battery_control(1,0);		//stop charging
							}
						set_loads(&load1_r, &load2_r, &load3_r, &load1_s, &load2_s, &load3_s);	//connect up loads
					}
				else /* Battery capacity is low, can charge*/
					{
						if (battery_c == 1)					//if battery is already charging
							{
								set_loads(&load1_r, &load2_r, &load3_r, &load1_s, &load2_s, &load3_s);	//connect up loads
							}
						else 
							{
								I_mains = 1 - (I_renewable - I_required);	//in amps
								battery_c = 1;								//
								battery_control(1,0);						//start charging
								set_pwm_vout(I_mains);
								set_loads(&load1_r, &load2_r, &load3_r, &load1_s, &load2_s, &load3_s);	//connect up loads
								controller(3, &I_mains, &mains_status, &load1_r, &load2_r, &load3_r, &load1_s, &load2_s, &load3_s, &I_renewable, &control);
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
								battery_d = 0;
								battery_control(0,1);		//stop discharging
							}
						if (battery_capacity > 480000)		//480000ms = 8 minutes
							{
								I_mains = (I_required - I_renewable);		//mains to supply the current deficit
								set_pwm_vout(I_mains);
								set_loads(&load1_r, &load2_r, &load3_r, &load1_s, &load2_s, &load3_s);	//connect up loads
								controller(2, &I_mains, &mains_status, &load1_r, &load2_r, &load3_r, &load1_s, &load2_s, &load3_s, &I_renewable, &control);
							}
						else 
							{
								if (battery_c != 1)
									{
										battery_c = 1;			//start charging
										battery_control(1,0);
									}
								I_mains = (I_required - I_renewable) + 1;	//enough to power the loads and charge the battery
								set_pwm_vout(I_mains);
								set_loads(&load1_r, &load2_r, &load3_r, &load1_s, &load2_s, &load3_s);	//connect up loads
								controller(1, &I_mains, &mains_status, &load1_r, &load2_r, &load3_r, &load1_s, &load2_s, &load3_s, &I_renewable, &control);
							}
					}
				else
					/* No, use mains and possible battery to supply deficit*/ 
					{
						if (battery_capacity > 180000)		//180000 ms = 3 minutes (3 mins is used so we have spare in case power goes down)
						/* if there is capacity, then discharge the battery */
							{
								if (battery_c == 1)
									{
										battery_c = 0;			//stop charging
										battery_control(1,0);
									}
								if (battery_d != 1)
									{
										battery_d = 1;
										battery_control(0,1);	//start discharging 
									}
								I_mains = I_required - I_renewable - 1;		//mains is fulfilling the remaining load current deficit
								set_pwm_vout(I_mains);
								set_loads(&load1_r, &load2_r, &load3_r, &load1_s, &load2_s, &load3_s);	//connect up loads
								controller(2, &I_mains, &mains_status, &load1_r, &load2_r, &load3_r, &load1_s, &load2_s, &load3_s, &I_renewable, &control);
							}
						else
						/* Not enough capacity in battery, so use mains to supply deficit*/
							{
								if (battery_d == 1)
									{
										battery_d = 0;
										battery_control(0,1);		//stop discharging
									}
								I_mains = I_required - I_renewable;	//mains is fulfilling the load current deficit
								if (((I_mains + 1.2) < 3) && (battery_capacity < 180000))		//if there is enough mains capacity left to charge battery
									{
										I_mains += 1;				//mains will also charge battery
										battery_c = 1;
										battery_control(1,0);
									}
								else if (battery_c == 1)
									{
										battery_c = 0;				//if it is charging, stop charging
										battery_control(1,0);
									}
								set_pwm_vout(I_mains);
								set_loads(&load1_r, &load2_r, &load3_r, &load1_s, &load2_s, &load3_s);	//connect up loads
								controller(1, &I_mains, &mains_status, &load1_r, &load2_r, &load3_r, &load1_s, &load2_s, &load3_s, &I_renewable, &control);
							}
					}
			}
	}

	return 0;
}

void init_usr_intfc()
{
	/*draws the main theme, sets up table*/
    //Clears screen, makes black, ready for data
    rectangle clear_screen = {.left=0, .right = (display.width-1), .top = 0, .bottom = display.height};
    fill_rectangle(clear_screen, BLACK);

    //TEAM H title bar
    rectangle top_bar = {.left=0, .right = (display.width-1), .top = 0, .bottom = 19};
    fill_rectangle(top_bar, WHITE);

    change_position(13,2);
    change_foreground(BLACK);
    change_background(WHITE);
    display_string("Team H");

	//Status frame
	rectangle outter_rect = {.left = 5, .right=display.width - 5, .top=top_bar.bottom + 13, .bottom=display.height - 5};
	fill_rectangle(outter_rect,WHITE);
	rectangle inner_rect = {.left=outter_rect.left + 2, .right=outter_rect.right - 2, .top=outter_rect.top + 2, .bottom=outter_rect.bottom - 2};
	fill_rectangle(inner_rect, BLACK);

	//Write "Status"
	change_position(13,23);
	change_foreground(WHITE);
	change_background(BLACK);
	display_string("Status");

	//Setting up tabling grid starting point
	init_table(inner_rect.left, inner_rect.top);

	//Writes the text
	update_table( 0,0, "C1:");
	update_table( 1,0, "C2:");
	update_table( 2,0, "C3:");

	update_table( 0,2, "S1:");
	update_table( 1,2, "S2:");
	update_table( 2,2, "S3:");

	update_table( 3,0, "B_Status:");
	update_table( 4,0, "B_Level:");
	update_table( 5,0, "%_Mains:");
	update_table( 6,0, "I_Mains:");
	update_table( 7,0, "I_Wind:");
	update_table( 8,0, "I_Solar:");
	update_table( 9,0, "BB_V:");
	update_table(10,0, "BB_C:");
	update_table(11,0, "Time:");
	update_table(12,0, "Pos:");	
	update_table(13,0, "I_Ren:");		
	update_table(14,0, "I_Req:");		

	update_table( 4,3, "mins");
	update_table( 5,4, "%");
	update_table( 6,4, "A");
	update_table( 7,4, "A");
	update_table( 8,4, "A");
	update_table( 9,4, "V");
	update_table(10,4, "A");
	update_table(11,5, "ms");
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

void set_pwm_vout(double current)
{
	/*Convert 0->10v (amplified out) to 0->255 */
	OCR2A = (int)(((current/3.0)*255)+0.5); //added 0.5, for x.y when 0<=y<=4 round down, 0<5 = round up.
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
	/* CHANGED IT BACK, FLIPPED LINES 496 AND 501 AS NOT USING MOSFETS ANYMORE*/
	if (val)
	{
		PORTOUT |= _BV(pin);
	}
	else
	{

		PORTOUT &= ~_BV(pin);
	}
}

/*
double get_time()
{
	//Getting timer time in terms of second
	return (counter/5);
}
*/

double get_v_amp() //NOTE: RMS value
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
  return (((bb_q_amp/512.0)*400)/sqrt(2)); //because max input at 400V.
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

void battery_stop(uint8_t mode, const uint32_t* start_time, uint32_t* total_time, uint8_t* battery_c, uint8_t* battery_d )
{
  //end charging
  if(mode)
  {
    *total_time += (counter-*start_time); //add the delta time.
    set_digital(CBATT,0); //stop charging
    *battery_c=0; //indicating stop charging
  }
  //end discharging
  else
  {
    *total_time -= (counter-*start_time); //minus the delta time after discharging.
    set_digital(DBATT,0);//stop discharging
    *battery_d=0; //indicating stop discharging
  }
}

void battery_start(uint8_t mode, uint32_t* start_time, uint8_t* battery_c, uint8_t* battery_d)
{
  //start charging
  if(mode)
  {
    set_digital(CBATT,1); //start charging
    set_digital(DBATT,0); //stop discharging
    *battery_c=1;
    *battery_d=0;
  }
  //start discharging
  else
  {
    set_digital(CBATT,0); //stop charging
    set_digital(DBATT,1); //start discharging
    *battery_c=0;
    *battery_d=1;
  }

  *start_time = counter;
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
	sprintf(sprintfBuff, "%5s", dataToStrBuff);		//formats array of chars into suitable format for display
	update_table(row, col, sprintfBuff);
}

void update_values(double* bb_v, double* bb_c, uint8_t* load1_r, uint8_t* load2_r, uint8_t* load3_r, uint8_t* load1_s, uint8_t* load2_s, uint8_t* load3_s, uint8_t* battery_c, uint8_t* battery_d, double* I_mains, double* I_wind, double* I_solar)
{	
	double battery_mins = 0;
	battery_mins = battery_capacity/60000.0;							//Getting battery capacity in minutes
	printNumber(&battery_mins, dataToStrBuff, sprintfBuff, 4,2);		//Update battery capacity value
	
	uint8_t I_mains_percentage;
	I_mains_percentage = ((*I_mains * 100.0)/3.0);
	//printNumber(&I_mains_percentage, dataToStrBuff, sprintfBuff, 5,2);	//Update mains percenatge current value
	
	printNumber(I_mains, dataToStrBuff, sprintfBuff, 6,2);				//Update mains current value
	
	printNumber(I_wind, dataToStrBuff, sprintfBuff, 7,2);				//Update wind current value
	printNumber(I_solar, dataToStrBuff, sprintfBuff, 8,2);				//Update solar current value
	
	printNumber(bb_v, dataToStrBuff, sprintfBuff, 9,2);					//Update voltage value
	printNumber(bb_c, dataToStrBuff, sprintfBuff, 10,2);				//Update current value
	
	double temp = 0;
	temp = (double)counter;
	printNumber(&temp, dataToStrBuff, sprintfBuff, 11,2);				//Update time
	
	(*load1_r) ? update_table(0,1, "Yes") : update_table(0,1, "No ");	//Update load 1 request
	(*load2_r) ? update_table(1,1, "Yes") : update_table(1,1, "No ");	//Update load 2 request
	(*load3_r) ? update_table(2,1, "Yes") : update_table(2,1, "No ");	//Update load 3 request

	(*load1_s) ? update_table(0,3, "Yes") : update_table(0,3, "No ");	//Update load 1 request
	(*load2_s) ? update_table(1,3, "Yes") : update_table(1,3, "No ");	//Update load 2 request
	(*load3_s) ? update_table(2,3, "Yes") : update_table(2,3, "No ");	//Update load 3 request

	if ((*battery_c == 0) && (*battery_d == 0))
		update_table(3,2, "     Idle");
	else if (*battery_c == 1)
		update_table(3,2, "   Charge");
	else
		update_table(3,2, "Discharge");
}

void battery_control(uint8_t charge_control, uint8_t discharge_control)
{
	if (charge_control == 1)
		{
			if (battery_c == 1)
				{
					set_digital(CBATT, battery_c);		//start charging the battery
					charge_start_time = counter;		//record charging start time
				}
			if (battery_c == 0)
				{
					set_digital(CBATT, battery_c);		//stop charging the battery
					charge_end_time = counter;			//record charging end time
					battery_capacity += (charge_end_time - charge_start_time);
				}
		}
		
	if (discharge_control == 1)
		{
			if (battery_d == 1)
				{
					set_digital(DBATT, battery_d);		//start discharging the battery
					discharge_start_time = counter;		//record discharging start time
				}
			if (battery_d == 0)
				{
					set_digital(DBATT, battery_d);		//stop discharging the battery
					discharge_end_time = counter;		//record discharging end time
					battery_capacity -= (discharge_end_time - discharge_start_time);
				}			
		}
}

void set_loads(uint8_t* load1_r, uint8_t* load2_r, uint8_t* load3_r, uint8_t* load1_s, uint8_t* load2_s, uint8_t* load3_s)
{
	*load1_s = *load1_r;	//give the set variables the value of the requests
	*load2_s = *load2_r;
	*load3_s = *load3_r;	
	set_digital(SLOAD1, *load1_s);
	set_digital(SLOAD2, *load2_s);
	set_digital(SLOAD3, *load3_s);
}

void set_loads_control(uint8_t* load1_s, uint8_t* load2_s, uint8_t* load3_s)
{
	set_digital(SLOAD1, *load1_s);
	set_digital(SLOAD2, *load2_s);
	set_digital(SLOAD3, *load3_s);
}

void controller(uint8_t mode, double* I_mains, uint8_t* mains_status, uint8_t* load1_r, uint8_t* load2_r, uint8_t* load3_r, uint8_t* load1_s, uint8_t* load2_s, uint8_t* load3_s, double* I_renewable, uint8_t* control)
{
	update_table(12,1, " Controller ");	
	double voltage_prev  = 0;
	double voltage_after = 0;
	voltage_prev = get_v_amp();		//obtain busbar voltage RMS
	
	double current_left = 0;
	
	if (voltage_prev < 238) 		//can change to improve control at a later stage
		{
			*I_mains += 0.1;						//increase mains output current by a small amount
			set_pwm_vout(*I_mains);					//apply this to the mains by the PWM signal
			voltage_after = get_v_amp();			//obtain new voltage value
			if (!(voltage_after > voltage_prev))	//test if the mains works
				{
					*mains_status = 0;				
					switch (mode) 
						{
							case 1 :	//starting point for situation 1 and 3
										if (battery_c == 1)
											{
												battery_c = 0;						//if charging, then stop charging
												battery_control(1,0);
											}										//no 'break' means that it follows case 2 code
						
							case 2 :	//starting point for situation 2 and 4
										if (battery_capacity > 10)
											{
												current_left = *I_renewable + 1;	//available current = renewables + battery
												if (battery_d == 0)
													{
														battery_d = 1;				//signal to discharge battery
														battery_control(0,1);		//discharge battery													
													}
											}
										else 
											{
												current_left = *I_renewable;
												if (battery_d == 1)
													{
														battery_d = 0;
														battery_control(0,1);		//if discharging, stop
													}
											}
										*load1_s = 0;								//set all loads to zero (not applied)
										*load2_s = 0;
										*load3_s = 0;
										if (*load1_r == 1)							//operating theeatre
											{
												 if (current_left > I1)				//if enough current
													{	
														*load1_s = 1;				//turn on load
														current_left -= I1;			//update available current
													}	
											}
										if (*load2_r == 1)							//life support
											{
												 if (current_left > I2)				//if enough current
													{	
														*load2_s = 1;				//turn on load
														current_left -= I2;			//update available current
													}													
											}
										if (*load3_r == 1)							//ward power
											{
												 if (current_left > I1)				//if enough current
													{	
														*load1_s = 1;				//turn on load
													}													
											}
										set_loads_control(load1_s, load2_s, load3_s);	//update the load switches										
										break;
							
							case 3 :	//starting point for situation 5
										battery_c = 0;
										battery_control(1,0);
										break;
						}
				}
			else
				{
					*mains_status = 1;	
				}
			*I_mains -= 0.1;			//bring mains back to the original level
			set_pwm_vout(*I_mains);		//apply this to the mains by the PWM signal
										//move this to the else loop above, if we go ahead with the proportional control
		}
	*control = 1;
}



