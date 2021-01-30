/******************************************************

A Simple Device Timer project designed using ATmega8
AVR MVU. The Timer is usefull for keeping a device
"ON" for a specific period of time. After the set time
elapse the timer automatically turns the load off.

The Timer uses a standart 16x2 lcd module for user interface
UI. User can set the time using a 3 button keypad.

After that Timer is started. While count down is in
progress, the time left is displayed on screen.

The program use our LCD driver library more details
of which can be found in Website.

Use avr-gcc + AVR Studio to compile.

Author: Avinash Gupta
E:Mail: me@avinashgupta.com
Web: www.eXtremeElectronics.co.in

*** THIS PROJECT IS PROVIDED FOR EDUCATION/HOBBY USE ONLY  ***

*** NO PROTION OF THIS WORK CAN BE USED IN COMMERIAL       ***
*** APPLICATION WITHOUT WRITTEN PERMISSION FROM THE AUTHOR ***

EVERYONE IS FREE TO POST/PUBLISH THIS ARTICLE IN
PRINTED OR ELECTRONIC FORM IN FREE/PAID WEBSITES/MAGAZINES/BOOKS
IF PROPER CREDIT TO ORIGINAL AUTHOR IS MENTIONED WITH LINKS TO
ORIGINAL ARTICLE




Copyright (C) 2008-2009 eXtreme Electronics, India.

******************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>

#include "lcd.h"

//Connection of Load
#define LOAD_DDR DDRC
#define LOAD_PORT PORTC
#define LOAD_POS PC0

//Global variable for the clock system
volatile unsigned int 	clock_millisecond=0;
volatile char 			clock_second=0;
volatile char 			clock_minute=0;
volatile char 			clock_hour=0;


void Wait(uint8_t n)
{

	uint8_t i,temp;
	temp=n*28;

	for(i=0;i<temp;i++)
	_delay_loop_2(0);
}

void LoadOn()
{
	LOAD_PORT|=(1<<LOAD_POS);
}

void LoadOff()
{
	LOAD_PORT&=(~(1<<LOAD_POS));
}
main()
{

	while(1)
	{
		LOAD_DDR|=(1<<LOAD_POS);

		LoadOff();

		//Enable Pullups on Keypad
		PORTB|=((1<<PB2)|(1<<PB1)|(1<<PB0));

		int8_t hr,min;	//Target Time
		hr=min=0;

		//Initialize the LCD Subsystem
		InitLCD(0);
		//Clear the display
		LCDClear();

		//Set up the timer1 as described in the
		//tutorial
		TCCR1B=(1<<WGM12)|(1<<CS11)|(1<<CS10);
		OCR1A=250;

		//Enable the Output Compare A interrupt
		TIMSK|=(1<<OCIE1A);

		//Enable interrupts globally
		sei();

		LCDClear();
		LCDWriteString("    Welcome     ");
		LCDWriteStringXY(0,1,"   Relay Timer  ");

		Wait(4);

		LCDClear();
		LCDWriteString("Set Time - 00:00");
		LCDWriteStringXY(0,1," Start     ^");

		uint8_t selection=1;
		uint8_t old_pinb=PINB;

		while(1)
		{
			while((PINB & 0b00000111) == (old_pinb & 0b00000111));
			
			//Input received


			if(!(PINB & (1<<PINB2)) && (old_pinb & (1<<PB2)))
			{
				//Selection key Pressed
				selection++;
				if(selection==3)
				selection =0;
			}

			if(!(PINB & (1<<PINB1)) && (old_pinb & (1<<PB1)))
			{
				//Up Key Pressed
				if(selection == 1)
				{
					
					//Hour is selected so increment it
					hr++;

					if(hr == 100)
					hr =0;
				}

				if(selection == 2)
				{

					//Min is selected so increment it
					min++;

					if(min == 60)
					min =0;
				}

				if(selection == 0)
				{
					//Start Selected
					break;
				}


			}

			if(!(PINB & (1<<PINB0)) && (old_pinb & (1<<PB0)))
			{
				//Down Key Pressed
				if(selection == 1)
				{

					//Hour is selected so decrement it
					hr--;

					if(hr == -1)
					hr =99;
				}

				if(selection == 2)
				{

					//Min is selected so decrement it
					min--;

					if(min == -1)
					min =59;
				}

				if(selection == 0)
				{
					//Start Selected
					break;
				}

			}

			
			old_pinb=PINB;

			

			//Update Display
			LCDClear();
			LCDWriteString("Set Time - 00:00");
			LCDWriteStringXY(0,1," Start    ");

			//Hour
			LCDWriteIntXY(11,0,hr,2);

			//Minute
			LCDWriteIntXY(14,0,min,2);

			if(selection == 0)
			LCDWriteStringXY(0,1,">");
			
			if(selection == 1)
			LCDWriteStringXY(11,1,"^");

			if(selection == 2)
			LCDWriteStringXY(14,1,"^");

			_delay_loop_2(0);
			_delay_loop_2(0);
			_delay_loop_2(0);
			_delay_loop_2(0);

			_delay_loop_2(0);
			_delay_loop_2(0);
			_delay_loop_2(0);
			_delay_loop_2(0);
		}

		//Start the Load
		LoadOn();

		//Now start the timer
		clock_hour = hr;
		clock_minute = min;
		clock_second =0;

		LCDClear();
		LCDWriteString("  Power Off In ");

		while(1)
		{
			LCDWriteIntXY(4,1,clock_hour,2);
			LCDWriteString(":");
			LCDWriteIntXY(7,1,clock_minute,2);
			LCDWriteString(":");
			LCDWriteIntXY(10,1,clock_second,2);

			if((clock_hour == 0) && (clock_minute == 0) && (clock_second == 0))
			{
				//Time Out
				LoadOff();

				LCDClear();
				LCDWriteString("Load Turned Off");
				
				while(1)
				{
					LCDWriteStringXY(0,1,"*Press Any Key*");

					Wait(1);

					LCDWriteStringXY(0,1,"                ");

					Wait(1);

					if((~PINB) & 0b00000111)
					break;

				}

				break;

				

			}

			_delay_loop_2(0);
			_delay_loop_2(0);
			_delay_loop_2(0);
			_delay_loop_2(0);
		}
		//Continue again
	}


}

//The output compate interrupt handler
//We set up the timer in such a way that
//this ISR is called exactly at 1ms interval
ISR(TIMER1_COMPA_vect)
{
	clock_millisecond++;
	if(clock_millisecond==1000)
	{
		clock_second--;
		clock_millisecond=0;
		if(clock_second==-1)
		{
			clock_minute--;
			clock_second=59;

			if(clock_minute==-1)
			{
				clock_hour--;
				clock_minute=59;
			}
		}
	}
}
