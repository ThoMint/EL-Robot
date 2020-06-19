/*
 *	main.c
 *	Created:	02.04.2020
 *	Author:		Thomas Hofmann
 */

#include "EL-Robot.h"
#include "LCD_MCP23008_16MHz.h"

int main(void)
{
	//Initialization
	set_fuses();			//JTAG disable and CPU-Speed to 16MHz
	init();					//Init timers and set pin modes
	PORTC |= IO_RESET;		//Port expander on
	TWBR = 12;				//TWBR=12, TWPS=0, TWSR per default,f_SCL = 400 kHz
	I2C_init();				//I2C Initialization
	sei();					//Global interrupt enable
	
	//LCD_init();
	//LCD_cmd(LCD_INIT);
	//LCD_cmd(LCD_CLEAR);
	
	lineFollower_init();
	
    while(1)
    {	
		//driveSpeed(50,50);				//Drive the robot with 50mm/s
		//driveDistance(200,200,200,200);	//Drive the robot with 200mm/s for 200mm
		followLine();
		showBatteryStatus();
	}
}