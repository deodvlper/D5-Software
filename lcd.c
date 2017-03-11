/*  Author: Steve Gunn
 * Licence: This work is licensed under the Creative Commons Attribution License.
 *           View this license at http://creativecommons.org/about/licenses/
 */
 
#include "ili934x.h"
#include "font.h"
#include "lcd.h"

lcd display = {LCDWIDTH, LCDHEIGHT, North, 0, 0, WHITE, BLACK};

void init_lcd()
{
	/* Disable JTAG in software, so that it does not interfere with Port C  */
	/* It will be re-enabled after a power cycle if the JTAGEN fuse is set. */
	MCUCR |= (1<<JTD);
	MCUCR |= (1<<JTD);
	
	/* Configure ports */
	CTRL_DDR = 0x7F;
	DATA_DDR = 0xFF;
	
	init_display_controller();
}

void set_orientation(orientation o)
{
	display.orient = o;
	write_cmd(MEMORY_ACCESS_CONTROL);
	if (o==North) { 
		display.width = LCDWIDTH;
		display.height = LCDHEIGHT;
		write_data(0x48);
	}
	else if (o==West) {
		display.width = LCDHEIGHT;
		display.height = LCDWIDTH;
		write_data(0xE8);
	}
	else if (o==South) {
		display.width = LCDWIDTH;
		display.height = LCDHEIGHT;
		write_data(0x88);
	}
	else if (o==East) {
		display.width = LCDHEIGHT;
		display.height = LCDWIDTH;
		write_data(0x28);
	}
	write_cmd(COLUMN_ADDRESS_SET);
	write_data16(0);
	write_data16(display.width-1);
	write_cmd(PAGE_ADDRESS_SET);
	write_data16(0);
	write_data16(display.height-1);
}

void fill_rectangle(rectangle r, uint16_t col)
{
	uint16_t x, y;
	write_cmd(COLUMN_ADDRESS_SET);
	write_data16(r.left);
	write_data16(r.right);
	write_cmd(PAGE_ADDRESS_SET);
	write_data16(r.top);
	write_data16(r.bottom);
	write_cmd(MEMORY_WRITE);
	for(x=r.left; x<=r.right; x++)
		for(y=r.top; y<=r.bottom; y++)
			write_data16(col);
}

void fill_rectangle_indexed(rectangle r, uint16_t* col)
{
	uint16_t x, y;
	write_cmd(COLUMN_ADDRESS_SET);
	write_data16(r.left);
	write_data16(r.right);
	write_cmd(PAGE_ADDRESS_SET);
	write_data16(r.top);
	write_data16(r.bottom);
	write_cmd(MEMORY_WRITE);
	for(x=r.left; x<=r.right; x++)
		for(y=r.top; y<=r.bottom; y++)
			write_data16(*col++);
}

void clear_screen()
{
	display.x = 0;
	display.y = 0;
	rectangle r = {0, display.width-1, 0, display.height-1};
	fill_rectangle(r, display.background);
}

void display_char(char c)
{
	uint16_t x, y;
	PGM_P fdata; 
	uint8_t bits, mask;
	uint16_t sc=display.x, ec=display.x + 9, sp=display.y, ep=display.y + 7; //e=end, s=start, c=col, p=row, => ec=display.x+9 : accomodate the width change. ep doesn't change, because changed at write_data and offset in the write_data_16
	if (c < 32 || c > 126) return; //limiting the type of characters that can be displayed.
	fdata = (c - ' ')*5 + font5x7; //remove offset based on the space.
	write_cmd(PAGE_ADDRESS_SET);
	write_data16(sp);
	write_data16(ep+8); //modified to +8 to accommodate 16 bits high.
	for(x=sc; x<=ec; x++) {
		write_cmd(COLUMN_ADDRESS_SET);
		write_data16(x); //I THINK NEED TO WRITE DATA 2X BECAUSE IT IS 16 BIT DATA. AND THE DATA PORT ONLY 8 bits
		write_data16(x);
		write_cmd(MEMORY_WRITE);
		bits = pgm_read_byte( ((x-sc)%2) ? fdata++ : fdata); //to do 2* width so, if postion relative to starting odd when modulus, use fdata then add, otherwise use fdata don't add.
		for(y=sp, mask=0x01; y<=ep; y++, mask<<=1)
			{
			write_data16((bits & mask) ? display.foreground : display.background);
			write_data16((bits & mask) ? display.foreground : display.background);
			}
	}

	//Double the inter-char spacing -- make function so it's cleaner.
	add_spacing(x,sp,ep);
	x++; //the next space
	add_spacing(x,sp,ep);

	display.x += 12; //because having 10 bits and having 2 spacing, should start the next bit at =9+2+1=12
	if (display.x >= display.width) { display.x=0; display.y+=16; } //changed from display.y+=8 to +=16 //note double the height.
}

void display_string(char *str)
{
	uint8_t i;
	for(i=0; str[i]; i++) 
		display_char(str[i]);
}

void add_spacing(uint16_t x, uint16_t sp, uint16_t ep)
{
	uint16_t y;
	write_cmd(COLUMN_ADDRESS_SET);
	write_data16(x);
	write_data16(x);
	write_cmd(MEMORY_WRITE);
	for(y=sp; y<=ep; y++)
		{
		write_data16(display.background);
		write_data16(display.background);
		}

}

void change_position(uint16_t x, uint16_t y)
{
	/*Change the position of the display to draw on. */
	if(x<0 || x>LCDWIDTH || y<0 || y>LCDHEIGHT)
	{return;} //if exceeding the max height/width then just return. do nothing.
	display.x=x;
	display.y=y;
}

void change_background(uint16_t colour)
{
	/* change the background color of the display. However, this only for the particular code after that it has been changed */
	display.background=colour;
}

void change_foreground(uint16_t colour)
{
	/*Changing foreground color for the code following it.*/
	display.foreground=colour;
}

void init_table(uint16_t x_str, uint16_t y_start )
{
	start_x = x_str+8; //offset from right one 1 pixel.
	start_y = y_start+10; //do down from the "status" text.
	//value_column = (display.width/2) - 16;
	//unit_column = (display.width*0.9);
}

void update_table(uint16_t row, uint16_t col, char *str)
{
	//start with 0,0 and for the row only 2 rows currently.
	//can add a protection in front of this.
	//change_position( (row) ? start_x : value_column ,start_y+row*16); //for x, either it's midpoint or it is the left starting point.
	//change_position((col) ? value_column : start_x,start_y + (row*16));
	switch (col) 
		{
			case 0 :	
						change_position(start_x ,      start_y + (row * 16) );	//first  argument is the x position (what column)
						break;													//second argument is the y position (what row)
			case 1 :	
						change_position((display.width*0.25) , start_y + (row * 16) );
						break;
			case 2 :	
						change_position((display.width*0.50) ,  start_y + (row * 16) );
						break;
			case 3 :	
						change_position((display.width*0.75) ,  start_y + (row * 16) );
						break;
			case 4 :	
						change_position((display.width*0.90) ,  start_y + (row * 16) );		//for the units
						break;
			case 5 :	
						change_position((display.width*0.85) ,  start_y + (row * 16) );		//for the ms
						break;
		}
	//the height always add 16 to the next one.
	display_string(str);
}

