#ifndef LCD_EL_ROBOT
#define LCD_EL_ROBOT

/////////////////////////////////////////////////////////////////////////////
//
// 32U4 + Portexpander MCP23008	+ LCD       	f_CPU = 16 MHz
//												f_SCL = 400 kHz
//
// ------------------------- I2C -------------------------------------------
// Beschaltung:
// LCD	MCP233008
// DB7	GP3
// DB6	GP2
// DB5	GP1
// DB4	GP0
// E    GP5
// RW	GND
// RS	GP4
//
//               f_CPU
// f_SCL = ---------------------  => TWBR=12 mit TWPS=0
//         16 + 2*TWBR * 4^TWPS
//
// Bem.: f_CPU muss min. 16*f_SCL sein
//       TWBR muss Wert > 10 haben
//
// MCP23008 Adr.: 0100 000  (A2=A1=A0=0)
//          BANK=1, Byte Mode
//
//
// ---------------------------- LCD ----------------------------------------
// Befehle: LCD_init(), LCD_cmd(char data), LCD_send(char data)
//          LCD_string(char *data)
//
// LCD_init();               initialisiert Port D
//                           und LCD im 4-Bit Mode, 2 Zeilen, 5x7 Dots
//                           Bsp.: LCD_init();
//
// LCD_cmd(char data);       schickt Befehl ans LCD
//                           Bsp.: LCD_cmd(0xC5);  //gehe zu 2. Zeile, 6. Position
//
// LCD_data(char data);      schickt Daten ans LCD
//                           Bsp.: LCD_data(0xEF); //sendet ein ö
//
// LCD_string(char *data);   schickt eine Zeichenkette ans LCD
//                           Bsp.: LCD_string("Hallo");    //sendet Hallo
//
////////////////////////////////////////////////////////////////////////////

#define I2C_ADDR 0b01000000								//MCP23008 Portexpander
#define E	5
#define RS	4
#define DB4	0
#define DB5	1
#define DB6	2
#define DB7	3

#define LCD_INIT	0x0C
#define LCD_CLEAR	0x01


#include <avr/io.h>
#include <stdint.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

void write_i2c(char addr, char data)
{
	_delay_ms(2);

	//////////////////////////////////// WRITE //////////////////////////////////////////////

	TWCR = TWCR|(1<<TWINT)|(1<<TWSTA)|(1<<TWEN);		//START
	while(!(TWCR&(1<<TWINT)));							//warten bis fertig

	TWDR = addr & ~(1<<0);								//Adr. xxxx xxxW + Write (W=0)
	TWCR = (1<<TWINT)|(1<<TWEN);						//senden
	while(!(TWCR&(1<<TWINT)));							//warten bis fertig

	TWDR = 0x0A;										//OLAT
	TWCR = (1<<TWINT)|(1<<TWEN);						//senden
	while(!(TWCR&(1<<TWINT)));							//warten bis fertig

	TWDR = data|(1<<E);									//E=1
	TWCR = (1<<TWINT)|(1<<TWEN);						//Data und E=1 senden
	while(!(TWCR&(1<<TWINT)));							//warten bis fertig

	TWDR = data &~(1<<E);								//E=0, LCD übernimmt bei neg. E-Flanke
	TWCR = (1<<TWINT)|(1<<TWEN);						//Data und E=0 senden
	while(!(TWCR&(1<<TWINT)));							//warten bis fertig

	TWCR = TWCR|(1<<TWINT)|(1<<TWSTO)|(1<<TWEN);		//STOP
}



void LCD_data(char data)								//RS=1, Daten
{
	char temp = data, x = 0xC0;							//C0 damit P7 und P6 vom PCF8574 = HIGH

	x = x | (1<<RS);									//SFR vom LCD mit RS auf Daten umschalten
	x = x | (1<<E);

	//Upper Nibble senden
	if (temp & 0b10000000) {x = x | (1<<DB7);}
	else {x = x & ~(1<<DB7);}

	if (temp & 0b01000000) {x = x | (1<<DB6);}
	else {x = x & ~(1<<DB6);}

	if (temp & 0b00100000) {x = x | (1<<DB5);}
	else {x = x & ~(1<<DB5);}

	if (temp & 0b00010000) {x = x | (1<<DB4);}
	else {x = x & ~(1<<DB4);}

	write_i2c(I2C_ADDR,x);								//Upper Nibble


	//Lower Nibble senden
	if (temp & 0b00001000) {x = x | (1<<DB7);}
	else {x = x & ~(1<<DB7);}

	if (temp & 0b00000100) {x = x | (1<<DB6);}
	else {x = x & ~(1<<DB6);}

	if (temp & 0b00000010) {x = x | (1<<DB5);}
	else {x = x & ~(1<<DB5);}

	if (temp & 0b00000001) {x = x | (1<<DB4);}
	else {x = x & ~(1<<DB4);}

	write_i2c(I2C_ADDR,x);								//Lower Nibble
}


void LCD_cmd(char cmd)									//RS=0, Befehle
{
	char temp = cmd, x = 0xC0;							//C0 damit P7 und P6 vom PCF8574 = HIGH

	x = x & ~(1<<RS);									//SFR vom LCD mit RS auf Befehle umschalten
	x = x |  (1<<E);

	//Upper Nibble senden
	if (temp & 0b10000000) {x = x | (1<<DB7);}
	else {x = x & ~(1<<DB7);}

	if (temp & 0b01000000) {x = x | (1<<DB6);}
	else {x = x & ~(1<<DB6);}

	if (temp & 0b00100000) {x = x | (1<<DB5);}
	else {x = x & ~(1<<DB5);}

	if (temp & 0b00010000) {x = x | (1<<DB4);}
	else {x = x & ~(1<<DB4);}

	write_i2c(I2C_ADDR,x);								//Upper Nibble

	//Lower Nibble senden
	if (temp & 0b00001000) {x = x | (1<<DB7);}
	else {x = x & ~(1<<DB7);}

	if (temp & 0b00000100) {x = x | (1<<DB6);}
	else {x = x & ~(1<<DB6);}

	if (temp & 0b00000010) {x = x | (1<<DB5);}
	else {x = x & ~(1<<DB5);}

	if (temp & 0b00000001) {x = x | (1<<DB4);}
	else {x = x & ~(1<<DB4);}

	write_i2c(I2C_ADDR,x);								//Lower Nibble
}




void LCD_init(void)
{
	char x = 0xC0;						//C0 damit P7 und P6 vom PCF8574 = HIGH

	_delay_ms(50);						//lt. Datenblatt min. 15ms nach Power ON warten

	/*
	//Function Set
	//DB7..DB4 = 0011
	x = x & (~(1<<DB7) & ~(1<<DB6));	//Interface auf 8 Bit
	x = x | (1<<DB5) | (1<<DB4);
	x = x &~(1<<RS);					//RS=0, E=1  (RW=0 per HW)
	x = x |(1<<E);
	write_i2c(I2C_ADDR,x);

	_delay_ms(10);						//lt. Datenblatt min 4.1ms warten

	//DB7..DB4 = 0011
	x = x & (~(1<<DB7) & ~(1<<DB6));	//Interface auf 8 Bit
	x = x | (1<<DB5) | (1<<DB4);
	x = x &~(1<<RS);
	x = x |(1<<E);
	write_i2c(I2C_ADDR,x);

	_delay_us(200);						//lt. Datenblatt min 100us warten

	//DB7..DB4 = 0011
	x = x & (~(1<<DB7) & ~(1<<DB6));	//Interface auf 8 Bit
	x = x | (1<<DB5) | (1<<DB4);
	x = x &~(1<<RS);
	x = x |(1<<E);
	write_i2c(I2C_ADDR,x);
	*/
	//DB7..DB4 = 0010
	x = x & (~(1<<DB7) & ~(1<<DB6) &~(1<<DB4));
	x = x | (1<<DB5);					//Interface auf 4 Bit
	x = x &~(1<<RS);
	x = x |(1<<E);
	write_i2c(I2C_ADDR,x);

	// 2-zeilig, 5x8 Matrix //
	//DB7..DB4 = 0010
	x = x & (~(1<<DB7) & ~(1<<DB6) &~(1<<DB4));
	x = x | (1<<DB5);					//Upper Nibble
	x = x &~(1<<RS);
	x = x |(1<<E);
	write_i2c(I2C_ADDR,x);

	//DB7..DB4 = 1000
	x = x | (1<<DB7);					//Lower Nibble
	x = x & (~(1<<DB6) & ~(1<<DB5) & ~(1<<DB4));
	x = x &~(1<<RS);
	x = x |(1<<E);
	write_i2c(I2C_ADDR,x);

	//Display Off //
	//DB7..DB4 = 0000
	x = x & (~(1<<DB7) & ~(1<<DB6) & ~(1<<DB5) & ~(1<<DB4)); //Upper Nibble
	x = x &~(1<<RS);
	x = x |(1<<E);
	write_i2c(I2C_ADDR,x);

	//DB7..DB4 = 1000
	x = x | (1<<DB7);					//Lower Nibble
	x = x & (~(1<<DB6) & ~(1<<DB5) & ~(1<<DB4));
	x = x &~(1<<RS);
	x = x |(1<<E);
	write_i2c(I2C_ADDR,x);

	//Clear Display //
	//DB7..DB4 = 0000
	x = x & (~(1<<DB7) & ~(1<<DB6) & ~(1<<DB5) & ~(1<<DB4)); //Upper Nibble
	x = x &~(1<<RS);
	x = x |(1<<E);
	write_i2c(I2C_ADDR,x);

	//DB7..DB4 = 0001
	x = x & (~(1<<DB7) & ~(1<<DB6) & ~(1<<DB5)); //Lower Nibble
	x = x | (1<<DB4);
	x = x &~(1<<RS);
	x = x |(1<<E);
	write_i2c(I2C_ADDR,x);

	//No Display Shift //
	//DB7..DB4 = 0000
	x = x & (~(1<<DB7) & ~(1<<DB6) & ~(1<<DB5) & ~(1<<DB4)); //Upper Nibble
	x = x &~(1<<RS);
	x = x |(1<<E);
	write_i2c(I2C_ADDR,x);

	//DB7..DB4 = 0011
	x = x & (~(1<<DB7) & ~(1<<DB6));	//Lower Nibble
	x = x | (1<<DB5) | (1<<DB4);
	x = x &~(1<<RS);
	x = x |(1<<E);
	write_i2c(I2C_ADDR,x);

	// Display ON , Cursor ON, Blinken ON //
	//DB7..DB4 = 0000
	x = x & (~(1<<DB7) & ~(1<<DB6) & ~(1<<DB5) & ~(1<<DB4)); //Upper Nibble
	x = x &~(1<<RS);
	x = x |(1<<E);
	write_i2c(I2C_ADDR,x);

	//DB7..DB4 = 1111
	x = x | (1<<DB7) | (1<<DB6) | (1<<DB5) | (1<<DB4);	//Lower Nibble
	x = x &~(1<<RS);
	x = x |(1<<E);
	write_i2c(I2C_ADDR,x);

}

void LCD_string(char *data)
{
	while (*data != '\0')				//bis zum letzten Zeichen
	{LCD_data(*data++);}
}

#endif	//LCD_EL_ROBOT