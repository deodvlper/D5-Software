#include <avr/io.h>
//#include <avr/iom644p.h> 		//Remove the error unresolved constant.
#include <avr/interrupt.h>
#include <avr/delay.h> 			//added for sleep.
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "lcd.h"
#define F_CPU 12000000UL 		//added to change CPU speed to 12MHz UL=unsigned long.

//DEFINITION: ADC
#define BBVOLTAGE PINA0 		//busbar voltage
#define BBCURRENT PINA1 		//busbar current
#define WTCURRENT PINA2			//wind current
#define PVCURRENT PINA3 		//solar current

//DEFINITION: DATA-PRITING -- dtostrf -- change PREC to change precision.
#define WD 3
#define PREC 1

//DEFINITION: DIGITAL INPUT
#define DDRIN DDRA
#define PORTIN PORTA
#define PININ PINA				//for reading inputs on port A
#define CLOAD1 PA4 				//call load
#define CLOAD2 PA5
#define CLOAD3 PA6

//DEFINITION: DIGITAL OUTPUT
#define DDROUT DDRD
#define PORTOUT PORTD			//Used for setting output pins on port D
#define PINOUT PIND	         	//Eg PINOUT = 0x28, 28(hex) in binary is 00101000, which is what is put on the port
#define CBATT PD0 				//Charge battery pin
#define DBATT PD1 				//Discharge battery pin
#define SLOAD1 PD2 				//Switch load 1
#define SLOAD2 PD3				//Switch load 2
#define SLOAD3 PD4				//Switch load 3

//DEFINITION: LOAD CURRENT LIST
#define I1 0.8	//operating theatre
#define I2 1.8	//life support
#define I3 1.4	//ward power

//DEFINITION: BUSBAR VOLTAGE LEVEL THRESHOLD (USED FOR CHECKING)
#define V_threshold 200

//DEFINITION CHARGE/DISCHARGE BATTERY
#define CHARGING 1
#define DISCHARGING 0

//FUNCTION LIST
void init_usr_intfc();			//Created function, draws the main theme, sets up table
void init_adc();				//Created function, enables ADC pins
void init_global_timer();  		//added to replace the global timer.
void init_pwm();				//Sets up the registers, for the voltage output pin
void init_digital();			//Sets up the digital inputs on port A, outputs on port D
void init_loads_pwm();			//Sets load switches and PWM output to zero
void set_pwm_vout(double current);
uint8_t get_digital(uint8_t pin);
void set_digital(uint8_t pin, uint8_t val);
//double get_time();			//get time is no longer required as you can just get the value of the global variable 'counter'
double get_v_amp(); //Note: Might mix both sampling to save time if needed.
double get_c_amp();
uint16_t read_adc(uint8_t channelNum);
void battery_stop(uint8_t mode, const uint32_t* start_time, uint32_t* total_time, uint8_t* battery_c, uint8_t* battery_d);
void battery_start(uint8_t mode, uint32_t* start_time, uint8_t* battery_c, uint8_t* battery_d);
void update_avg(const double* total_energy,const uint64_t* sample,double* avg_power);
void update_energy(const uint16_t* voltage_read, const uint16_t* current_read, uint64_t* sample, double* total_energy);
void printNumber(double* value, char* dataToStrBuff, char*sprintfBuff, uint8_t row, uint8_t col);
void update_values(double* bb_v, double* bb_c, uint8_t* load1_r, uint8_t* load2_r, uint8_t* load3_r, uint8_t* load1_s, uint8_t* load2_s, uint8_t* load3_s, double* i_mains, double* I_wind, double* I_solar, uint8_t* mains_status);		//Created function, updates the values
void battery_control(uint8_t charge_control, uint8_t discharge_control);
void set_loads(uint8_t* load1_r, uint8_t* load2_r, uint8_t* load3_r, uint8_t* load1_s, uint8_t* load2_s, uint8_t* load3_s);
void set_loads_control(uint8_t* load1_s, uint8_t* load2_s, uint8_t* load3_s);
void controller(uint8_t mode, double* I_mains, uint8_t* mains_status, uint8_t* load1_r, uint8_t* load2_r, uint8_t* load3_r, uint8_t* load1_s, uint8_t* load2_s, uint8_t* load3_s, double* I_renewable, uint8_t* control);							

