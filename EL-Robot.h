#ifndef EL_ROBOT
#define EL_ROBOT

/*
 *	Roboter.h
 *	Created:	02.04.2020
 *	Author:		WALC
 *	Modified:	Thomas Hofmann
 */ 
#define F_CPU 16000000UL

#include <avr/io.h>
#include <stdint.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

/************************************************************************/
/*	Global Variables                                                    */
/************************************************************************/

volatile unsigned long millis;						//For timing inside the program, is incremented every millisecond
volatile unsigned long rightWheelMicroS;			//Stores the microsS between two pulses right
volatile unsigned long leftWheelMicroS;				//Stores the microsS between two pulses right
volatile int rightWheelSpeed;						//Stores the current speed of the right wheel in mm/s
volatile int leftWheelSpeed;						//Stores the current speed of the left wheel in mm/s
volatile int rightWheelDist;						//Stores the distance traveled by the right wheel in mm
volatile int leftWheelDist;							//Stores the distance traveled by the left wheel in mm
volatile int rightWheelThrottle;					//Stores the current throttle of the right wheel from -100 to 100
volatile int leftWheelThrottle;						//Stores the current throttle of the left wheel from -100 to 100
volatile unsigned long nextTimedEventBatteryState;	//Stores the next time stamp the battery state should be updated in ms
volatile unsigned long nextTimedEventDriveSpeed;	//Stores the next time stamp the speed of the wheels should be updated in ms
//Line follower
volatile unsigned long nextTimedEventAccelLeftLF;	//Stores the next time stamp the speed of the left wheel should be updated
volatile unsigned long nextTimedEventAccelRightLF;	//Stores the next time stamp the speed of the right wheel should be updated
volatile int leftRefLF, rightRefLF;					//Stores the reference voltage for the background
volatile int leftThrLF, rightThrLF;					//Stores the current throttle of the wheels

/************************************************************************/
/*	Definitions PORTS                                                   */
/************************************************************************/

// PORTB
#define LED_RV			(1<<0)
#define LED_RH			(1<<1)
#define LED_LV			(1<<2)
#define LED_LH			(1<<3)
#define LED_GREEN		(1<<4)
#define LED_RED			(1<<5)
#define MOT_RIGHT		(1<<6)
#define BEEPER			(1<<7)

// PORTD
// PD0 -> I2C SCL
// PD1 -> I2C SDA
#define RECEIVER		(1<<2)
#define TRANSMITTER		(1<<3)
#define WHEEL_RIGHT		(1<<4)
#define DEBUG1			(1<<5)
#define LF_EMITTER		(1<<6)
#define MOT_LEFT		(1<<7)

//PORTF
#define MEASURE_UB		(1<<0)
#define ADC_UB			0
#define LF_LEFT			(1<<1)
#define ADC_LF_LEFT		1
#define LF_RIGHT		(1<<4)
#define ADC_LF_RIGHT	4
#define CHOOSE_LR		(1<<5)
#define CHOOSE_FB		(1<<6)
#define DEBUG2			(1<<7)

//PORTC
#define IO_RESET		(1<<6)
#define WHEEL_LEFT		(1<<7)

//PORTE
#define IO_INT			(1<<6)

/************************************************************************/
/*	Motor driver														*/
/************************************************************************/

#define M_THROTTLE_MAX  100
#define M_THROTTLE_STOP 0
#define M_THROTTLE_MIN -100

#define PWM_VOR_MAX		125		//2 ms
#define PWM_STOPP		94		//1,504 ms
#define PWM_RET_MAX		62		//0,992 ms

/************************************************************************/
/*	Defines Voltage LiPo												*/
/************************************************************************/

#define UREF1 973	//8.4V
#define UREF2 875	//7.6V
#define UREF3 756	//6.6V

/************************************************************************/
/*	Defines Line Follower Parameters									*/
/************************************************************************/

#define LF_ACCEL 50			//The time in ms in between accelerations, the lower the higher the acceleration
#define LF_TOP_SPEED 50		//The top speed of both wheels while following a line
#define LF_TOLERANCE 50		//The tolerance for the background getting darker without triggering

/************************************************************************/
/* Functions                                                           */
/************************************************************************/

//Macro to determine the absolute value of N
#define ABS(N) ((N<0)?(-N):(N))

//JTAG disable and CPU-PS to 16MHz
void set_fuses(void);

//Initialize the robot by setting pins to input/output and setting up the timer for the motor driver
void init(void);

//Set up the timer for the motor driver
void pwm_timer(void);

//Initialize Timers
void timer_init(void);

//Initialize Line follower
void lineFollower_init(void);

//Init the I2C Bus in order to communicate with the Port expander
void I2C_init(void);

//Map a integer number from a well known range to a new range
int map(int x, int in_min, int in_max, int out_min, int out_max);

//Drives the robots motors with a throttle from -100 to 100
void drive(int left, int right);

//Drives the robot with speed in mm/s
void driveSpeed(int leftSpeed, int rightSpeed);

//Drives robot with speed in mm/s and distance in mm
uint8_t driveDistance(int leftDist, int rightDist, int leftSpeed, int rightSpeed);

//Measures the voltage at the channel and returns a 10bit value (0-1024) (0-5V)
unsigned int adc_measure(unsigned char channel);

//Returns the current state of the battery
int getBatteryStatus(void);

//Displays the current state of the battery at the bi color led
void showBatteryStatus(void);

//Follows a line that is darker than the background
void followLine(void);

#endif	//EL_ROBOT