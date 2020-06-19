/*
 *	EL_Robot.c
 *	Created: 02.04.2020
 *	Author: Thomas Hofmann
 */ 

#include "EL-Robot.h"

void set_fuses(void)
{
	MCUCR |= (1<<JTD);
	MCUCR |= (1<<JTD);
	
	CLKPR = 0b10000000;
	CLKPR = 0;
	CLKPR = 0;
	CLKPR = 0;
	CLKPR = 0;
}

void init(void)
{
	DDRB |= LED_RV | LED_RH | LED_LV | LED_LH | LED_GREEN | LED_RED | MOT_RIGHT | BEEPER;
	PORTB = 0;
	
	DDRD |= TRANSMITTER | LF_EMITTER | DEBUG1 | MOT_LEFT;
	
	DDRF |= CHOOSE_LR | CHOOSE_FB | DEBUG2;
	
	DDRC |= IO_RESET;
	
	PORTE |= IO_INT;
	
	millis=0;
	rightWheelMicroS=0;
	leftWheelMicroS=0;
	rightWheelSpeed=0;
	leftWheelSpeed=0;
	rightWheelDist=0;
	leftWheelDist=0;
	rightWheelThrottle=0;
	leftWheelThrottle=0;
	nextTimedEventBatteryState=0;
	//Line follower
	nextTimedEventAccelLeftLF=0;
	nextTimedEventAccelRightLF=0;
	leftRefLF=0;
	rightRefLF=0;
	leftThrLF=0;
	rightThrLF=0;
	
	pwm_timer();
	timer_init();
}

void pwm_timer(void)
{
	//Configure Timer 4 as Fast PWM Mode
	TCCR4A = TCCR4A | (1<<PWM4B);
	TCCR4C = TCCR4C | (1<<PWM4D);
	TCCR4D = TCCR4D &~(1<<WGM41);
	TCCR4D = TCCR4D &~(1<<WGM40);		//Fast PWM on OC4B and OC4D
	
	TCCR4A = TCCR4A &~(1<<COM4B0);
	TCCR4A = TCCR4A | (1<<COM4B1);		//COM4B1:0=2

	TCCR4C = TCCR4C &~(1<<COM4D0);
	TCCR4C = TCCR4C | (1<<COM4D1);		//COM4D1:0=2

	TC4H = 0x03;
	OCR4C = 0xE8;						//f_PWM = f_CLK_T4/(1+OCR4C) = 62,5kHz/1000 = 62,5 Hz
	TC4H = 0x00;
	OCR4B = PWM_STOPP;					//Duty cycle on OC4B-Pin (PB6)
	OCR4D = PWM_STOPP;					//Duty cycle on OC4D-Pin (PD7)
	
	TCCR4B = TCCR4B | (1<<CS43);
	TCCR4B = TCCR4B &~(1<<CS42);
	TCCR4B = TCCR4B &~(1<<CS41);		//f_CLK_T4 = CLK_IO/Prescaler = 16MHz/256 = 62,5kHz
	TCCR4B = TCCR4B | (1<<CS40);		//Timer4 Prescaler = 1, Start PWM
}

void timer_init(void)
{
	//init Timer 0
	TIMSK0 |= (1<<TOIE0);			//Timer-Overflow-Interrupt-Enable
	TCCR0B |= (1<<CS10)|(1<<CS11);	//Prescaler
	TCNT0 = 0x06;					//Initial-Value
	
	//init Timer 1 Input Capture
	TIMSK1 |= ((1<<ICIE1)|(1<<TOIE1));	//Input Capture Interrupt and Timer Overflow Interrupt enable
	TCCR1A = 0x00;			//For PWM Output, disable all
	TIFR1 |= (1<<ICF1);		//Clear input capture flag, automatically cleared when ISR is executed
	TCCR1B |= (1<<ICNC1);	//input noise filter on
	TCCR1B &= ~(1<<ICES1);	//falling edge
	TCCR1B |= (1<<CS11);	//Prescaler 8
	TCNT1 = 0x600;			//Initial value to 1536
	
	//init Timer 3 Input Capture
	TIMSK3 |= ((1<<ICIE1)|(1<<TOIE1));	//Input Capture Interrupt and Timer Overflow Interrupt enable
	TCCR3A = 0x00;			//For PWM Output, disable all
	TIFR3 |= (1<<ICF3);		//Clear input capture flag, automatically cleared when ISR is executed
	TCCR3B |= (1<<ICNC3);	//input noise filter on
	TCCR3B &= ~(1<<ICES3);	//falling edge
	TCCR3B |= (1<<CS31);	//Prescaler 8
	TCNT3 = 0x600;			//Initial value to 1536
}

void lineFollower_init(void)
{
	PORTD |= LF_EMITTER;	//Turn on the Emitters
	_delay_ms(100);
	//Measure the reference of the brighter background
	leftRefLF = adc_measure(ADC_LF_LEFT);
	rightRefLF = adc_measure(ADC_LF_RIGHT);
}

void I2C_init(void)
{
	/////////////////////////////////////////////////////////////////////////////
	//
	// 32U4 + Port expander MCP23008 + LCD	     f_CPU = 16 MHz
	//                                           f_SCL = 400 kHz
	//
	//               f_CPU
	// f_SCL = ---------------------  => TWBR=12 with TWPS=0
	//         16 + 2*TWBR * 4^TWPS
	//
	// Bem.: f_CPU has to be min. 16*f_SCL
	//       TWBR has to be >10
	//
	// MCP23008 Address: 0100 000  (A2=A1=A0=0)
	//
	////////////////////////////////////////////////////////////////////////////

	///////////////// configure IOCON ///////////////////
	
	TWCR = TWCR|(1<<TWINT)|(1<<TWSTA)|(1<<TWEN);		//START
	while(!(TWCR&(1<<TWINT)));							//wait till ready

	TWDR = 0b01000000;									//Address 0100 000W + Write (W=0)
	TWCR = (1<<TWINT)|(1<<TWEN);						//send
	while(!(TWCR&(1<<TWINT)));							//wait till ready

	TWDR = 0x05;										//send Register address IOCON
	TWCR = (1<<TWINT)|(1<<TWEN);						//send Register address
	while(!(TWCR&(1<<TWINT)));							//wait till ready
	
	TWDR = 0x2A;										//configure IOCON: Byte Mode, Slew Rate enable, no Open Drain at INTn, INTn active-high
	TWCR = (1<<TWINT)|(1<<TWEN);						//send Konf.
	while(!(TWCR&(1<<TWINT)));							//wait till ready
	
	TWCR = TWCR|(1<<TWINT)|(1<<TWSTO)|(1<<TWEN);		//STOP
	

	///////////////// configure GP with IODIR as OUTPUT ///////////////////
	
	TWCR = TWCR|(1<<TWINT)|(1<<TWSTA)|(1<<TWEN);		//START
	while(!(TWCR&(1<<TWINT)));							//wait till ready

	TWDR = 0b01000000;									//Address 0100 000W + Write (W=0)
	TWCR = (1<<TWINT)|(1<<TWEN);						//send
	while(!(TWCR&(1<<TWINT)));							//wait till ready

	TWDR = 0x00;										//send Register address. IODIR
	TWCR = (1<<TWINT)|(1<<TWEN);						//send Register address
	while(!(TWCR&(1<<TWINT)));							//wait till ready
	
	TWDR = 0x00;										//configure IODIR, all Pins as OUTPUT
	TWCR = (1<<TWINT)|(1<<TWEN);						//send Konf.
	while(!(TWCR&(1<<TWINT)));							//wait till ready
	
	TWCR = TWCR|(1<<TWINT)|(1<<TWSTO)|(1<<TWEN);		//STOP
}

int map(int x, int in_min, int in_max, int out_min, int out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void drive(int left, int right)
{
	left = left * (-1);										//Flip left motor
	left = left>M_THROTTLE_MAX ? M_THROTTLE_MAX:left;		//Constrain left motor throttle from to 100
	left = left<M_THROTTLE_MIN ? M_THROTTLE_MIN:left;		//Constrain left motor throttle from to -100
	right = right>M_THROTTLE_MAX ? M_THROTTLE_MAX:right;	//Constrain right motor throttle from to 100
	right = right<M_THROTTLE_MIN ? M_THROTTLE_MIN:right;	//Constrain right motor throttle from to -100
	uint8_t pwm_links = map(left, M_THROTTLE_MIN, M_THROTTLE_MAX, PWM_RET_MAX, PWM_VOR_MAX);		//Map the value from -100 - 100 to 62 - 125
	uint8_t pwm_rechts = map(right, M_THROTTLE_MIN, M_THROTTLE_MAX, PWM_RET_MAX, PWM_VOR_MAX);	//Map the value from -100 - 100 to 62 - 125
	
	TCNT4H = 0;
	OCR4B = pwm_rechts;	//Set the pwm
	TCNT4H = 0;
	OCR4D = pwm_links;	//Set the pwm
}

void driveSpeed(int leftSpeed, int rightSpeed)
{	
	if(nextTimedEventDriveSpeed<millis)
	{
		if(leftSpeed>leftWheelSpeed)
		{
			leftWheelThrottle += 1;			//If the current speed is less than the desired speed, throttle up
		}
		else if(leftSpeed<leftWheelSpeed)
		{
			leftWheelThrottle -= 1;			//If the current speed is more than the desired speed, throttle down
		}
		
		if(rightSpeed>rightWheelSpeed)
		{
			rightWheelThrottle += 1;		//If the current speed is less than the desired speed, throttle up
		}
		else if(rightSpeed<rightWheelSpeed)
		{
			rightWheelThrottle -= 1;		//If the current speed is more than the desired speed, throttle down
		}
		
		leftWheelThrottle = leftWheelThrottle < -100 ? -100:leftWheelThrottle;	 //Constrain the throttle to -100
		leftWheelThrottle = leftWheelThrottle > 100 ? 100:leftWheelThrottle;	 //Constrain the throttle to 100
		
		rightWheelThrottle = rightWheelThrottle > 100 ? 100:rightWheelThrottle;	 //Constrain the throttle to 100
		rightWheelThrottle = rightWheelThrottle < -100 ? -100:rightWheelThrottle;//Constrain the throttle to -100
		
		drive(leftWheelThrottle,rightWheelThrottle);							 //Update the throttle to the motor driver
		//Calculate the update rate in order to not overdrive it, has to wait at least one encoder pulse or update every half a second at lower speeds
		unsigned int updateRate = (ABS(leftSpeed)<2 && ABS(rightSpeed)<2) ? 500:(207000/(((ABS(leftSpeed)+ABS(rightSpeed))/2)*15));
		nextTimedEventDriveSpeed = millis + updateRate;
	}
}

uint8_t driveDistance(int leftDist, int rightDist, int leftSpeed, int rightSpeed)
{
	leftWheelDist=0;
	rightWheelDist=0;
	while(1)
	{
		
		if((leftWheelDist<leftDist) && (rightWheelDist<rightDist))
		{
			//No wheel has reached the desired distance, drive with speed
			driveSpeed(leftSpeed,rightSpeed);
		}
		else if((leftWheelDist<leftDist) && !(rightWheelDist<rightDist))
		{
			//Stop right wheel, distance reached
			drive(leftWheelThrottle,0);
			driveSpeed(leftSpeed,0);
		}
		else if(!(leftWheelDist<leftDist) && (rightWheelDist<rightDist))
		{
			//Stop left wheel, distance reached
			drive(0,rightWheelThrottle);
			driveSpeed(0,rightSpeed);
		}
		else if(!(leftWheelDist<leftDist) && !(rightWheelDist<rightDist))
		{
			leftWheelThrottle=0;
			rightWheelThrottle=0;
			drive(0,0);
			driveSpeed(0,0);
			return 1;
		}
	}
	return 0;
}

unsigned int adc_measure(unsigned char channel)
{
	unsigned int result=0;
	
	ADMUX = 0;
	ADMUX &= ~(1<<REFS1)&~(1<<REFS0);			//Ext. AREF = 5V
	ADMUX &= ~(1<<ADLAR);						//10bit
	
	ADCSRB &= ~(1<<MUX5);
	ADMUX &= ~(1<<MUX4)&~(1<<MUX3)&~(1<<MUX2)&~(1<<MUX1);
	ADMUX |= channel;
	
	ADCSRA |= (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);	//ADC on, Prescaler to 128 -> 125kHz Samplingfrequency

	ADCSRA |= (1<<ADSC);						//Start
	while(ADCSRA&(1<<ADSC));					//Wait till the conversion is complete
	result = ADCW;
	return result;
}

int getBatteryStatus()
{
	//25 Samples, 1 average value
	unsigned int batteryVoltage = 0;
	for(int i=0;i<25;i++)
	{
		batteryVoltage+=adc_measure(0x00);
	}
	batteryVoltage/=25;
	
	if(batteryVoltage<=UREF1 && batteryVoltage>UREF2)
	{
		return 1;	//8.4V - 7.6V
	}
	else if(batteryVoltage<=UREF2 && batteryVoltage>UREF3)
	{
		return 2;	//7.6V - 6.6V
	}
	else if(batteryVoltage<=UREF3)
	{
		return 3;	// < 6.6V
	}else
	{
		return -1;	//Battery over voltage !!!
	}
}

void showBatteryStatus()
{
	if(millis > nextTimedEventBatteryState)
	{
		switch(getBatteryStatus())
		{
			case 1:
			PORTB |= (1<<PINB4);							//green LED on
			PORTB &= ~(1<<PINB5);							//red LED off
			nextTimedEventBatteryState = millis + 1000;		//update every second
			break;
			case 2:
			PORTB |= ((1<<PINB5)|(1<<PINB4));				//green and red LED on
			nextTimedEventBatteryState = millis + 1000;		//update every second
			break;
			case 3:
			PORTB &= ~(1<<PINB4);							//green LED off
			PORTB ^= (1<<PINB5);							//Toggle red LED on/off
			nextTimedEventBatteryState = (PORTB & (1<<PINB5)) ? (millis + 100) : (millis + 4000);	//100ms on, 4000ms off
			break;
		}
	}
}

void followLine(void)
{
	int left = adc_measure(ADC_LF_LEFT);
	int right = adc_measure(ADC_LF_RIGHT);
	
	if(left<leftRefLF-LF_TOLERANCE)		//Darker line than background detected
	{
		//rightThr = 15;
		if(millis>nextTimedEventAccelRightLF)
		{
			rightThrLF++;	//Accelerate slowly, otherwise the robot does a wheelie and loses track
			nextTimedEventAccelRightLF=millis+LF_ACCEL;
		}
		rightThrLF = rightThrLF>LF_TOP_SPEED ? LF_TOP_SPEED:rightThrLF;
	}
	else
	{
		rightThrLF = 0;	//Stop
	}
	
	if(right<rightRefLF-LF_TOLERANCE)	//Darker line than background detected
	{
		//leftThr = 15;
		if(millis>nextTimedEventAccelLeftLF)
		{
			leftThrLF++;	//Accelerate slowly, otherwise the robot does a wheelie and loses track
			nextTimedEventAccelLeftLF=millis+LF_ACCEL;
		}
		leftThrLF = leftThrLF>LF_TOP_SPEED ? LF_TOP_SPEED:leftThrLF;
	}
	else
	{
		leftThrLF = 0;	//stop
	}
	
	drive(leftThrLF,rightThrLF);
}

//Triggered when timer 0 overflows every millisecond, millis is used for timing of different tasks
ISR(TIMER0_OVF_vect)
{
	millis++;
	TCNT0 = 0x06;
}

//Triggered when the timer of the right wheel overflows, add 32 ms to the measured time
ISR(TIMER1_OVF_vect)
{
	rightWheelMicroS += 32000;
	TCNT1 = 0x600;	//Initial value to 1536
}

//Triggered when the timer of the left wheel overflows, add 32 ms to the measured time
ISR(TIMER3_OVF_vect)
{
	leftWheelMicroS += 32000;
	TCNT3 = 0x600;	//Initial value to 1536
}

//Input Capture ISR
//Triggered when the right wheel detects a falling edge
//Used to measure time and calculate speed, travel a certain distance
ISR(TIMER1_CAPT_vect)
{
	//Calculate speed and traveled distance
	rightWheelMicroS += (TCNT1-0x600)/2;
	rightWheelMicroS *= 15;
	rightWheelSpeed = (207 * 1000000) / rightWheelMicroS;
	rightWheelDist += 14;
	rightWheelMicroS = 0;
	
	//Choose the sign of the motor depending on the throttle, not accurate at lower speeds
	//Problem: no HW way to determine the direction of the wheel
	rightWheelSpeed = rightWheelSpeed * (rightWheelThrottle<0 ? -1:1);
	
	TCNT1 = 0x600;	//Initial value to 1536
	//Reset the timer overflow flag register of timer 1
	//just in case if it has overflow during speed calculation
	TIFR1 |= (1<<TOV1);	
}

//Input Capture ISR
//Triggered when the left wheel detects a falling edge
//Used to measure time and calculate speed, travel a certain distance
ISR(TIMER3_CAPT_vect)
{
	//Calculate speed and traveled distance
	leftWheelMicroS += (TCNT3-0x600)/2;
	leftWheelMicroS *= 15;
	leftWheelSpeed = (207 * 1000000) / leftWheelMicroS;
	leftWheelDist += 14;
	leftWheelMicroS = 0;
	
	//Choose the sign of the motor depending on the throttle, not accurate at lower speeds
	//Problem: no HW way to determine the direction of the wheel
	leftWheelSpeed = leftWheelSpeed * (leftWheelThrottle<0 ? -1:1);
	
	TCNT3 = 0x600;	//Initial value to 1536
	//Reset the timer overflow flag register of timer 3
	//just in case if it has overflow during speed calculation
	TIFR3 |= (1<<TOV3);	
}