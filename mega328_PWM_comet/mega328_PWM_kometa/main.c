#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdint.h>


unsigned char vpred_ON, vpred_OFF, dozadu_ON, dozadu_OFF, change, selection, kometa_ON, kometa_OFF = 0;
unsigned char vlna_ON, XA, XB = 0;
unsigned char PWM1, PWM2, PWM3, PWM4, PWM5, PWM6, PWMXA, PWMXB = 0; //smer
unsigned char duty0A, duty0B, duty1A, duty1B, duty2A, duty2B = 0; //hodnoty PWM
unsigned char kom_ispeed = 0;
unsigned int OVF_update = 0;
unsigned int kometa_count = 0;
int _speed = 0;


unsigned char pwm_table[256] ={0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02,
	0x02, 0x02, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x04, 0x04, 0x04, 0x04, 0x04, 0x05, 0x05, 0x05,
	0x05, 0x06, 0x06, 0x06, 0x07, 0x07, 0x07, 0x08, 0x08, 0x08, 0x09, 0x09, 0x0A, 0x0A, 0x0B, 0x0B,
	0x0C, 0x0C, 0x0D, 0x0D, 0x0E, 0x0F, 0x0F, 0x10, 0x11, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
	0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1F, 0x20, 0x21, 0x23, 0x24, 0x26, 0x27, 0x29, 0x2B, 0x2C,
	0x2E, 0x30, 0x32, 0x34, 0x36, 0x38, 0x3A, 0x3C, 0x3E, 0x40, 0x43, 0x45, 0x47, 0x4A, 0x4C, 0x4F,
	0x51, 0x54, 0x57, 0x59, 0x5C, 0x5F, 0x62, 0x64, 0x67, 0x6A, 0x6D, 0x70, 0x73, 0x76, 0x79, 0x7C,
	0x7F, 0x82, 0x85, 0x88, 0x8B, 0x8E, 0x91, 0x94, 0x97, 0x9A, 0x9C, 0x9F, 0xA2, 0xA5, 0xA7, 0xAA,
	0xAD, 0xAF, 0xB2, 0xB4, 0xB7, 0xB9, 0xBB, 0xBE, 0xC0, 0xC2, 0xC4, 0xC6, 0xC8, 0xCA, 0xCC, 0xCE,
	0xD0, 0xD2, 0xD3, 0xD5, 0xD7, 0xD8, 0xDA, 0xDB, 0xDD, 0xDE, 0xDF, 0xE1, 0xE2, 0xE3, 0xE4, 0xE5,
	0xE6, 0xE7, 0xE8, 0xE9, 0xEA, 0xEB, 0xEC, 0xED, 0xED, 0xEE, 0xEF, 0xEF, 0xF0, 0xF1, 0xF1, 0xF2,
	0xF2, 0xF3, 0xF3, 0xF4, 0xF4, 0xF5, 0xF5, 0xF6, 0xF6, 0xF6, 0xF7, 0xF7, 0xF7, 0xF8, 0xF8, 0xF8,
	0xF9, 0xF9, 0xF9, 0xF9, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFB, 0xFB, 0xFB, 0xFB, 0xFB, 0xFB, 0xFC,
	0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD,
0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFF, 0xFF};

// program komety (pauzi)
unsigned int kometa_speed[100]={
 3, 200, 1000, 160, 500, 1000, 400, 2000, 600, 800,
 600, 2000, 100, 1000, 500, 160, 500, 200, 1000, 160,
 300, 100, 700, 900, 5000, 400, 2000, 100, 1000, 5000,
 2000, 300, 500, 2000, 5000, 200, 1000, 600, 200, 100,
 6000, 200, 5000, 200, 2000, 1000, 5000, 2000, 100, 2000,
 4000, 300, 600, 800, 400, 2000, 1000, 300, 1000, 200,
 300, 100, 700, 900, 5000, 400, 2000, 100, 1000, 5000,
 2000, 300, 500, 2000, 5000, 200, 1000, 600, 200, 100,
 6000, 200, 5000, 200, 2000, 1000, 5000, 2000, 100, 2000,
 300, 100, 700, 900, 5000, 400, 2000, 100, 1000, 5000,
};

void tim0_int(){
	DDRD |= (1 << DDD5) | (1 << DDD6);
	//TCCR0A |= (1 << COM0A0) | (1 << COM0A1) | (1 << COM0B0) | (1 << COM0B1);
	TCCR0A |= (1 << COM0A1) | (1 << COM0B1);
	TCCR0A |= (1 << WGM00);
	TCCR0B |= (1 << CS00) | (1 << CS01);
	OCR0A = 0;
	OCR0B = 0;
}


void tim1_int(){
	DDRB |= (1 << DDB1)|(1 << DDB2);
	//TCCR1A |= (1 << COM1A0) | (1 << COM1A1) | (1 << COM1B0) | (1 << COM1B1);
	TCCR1A |= (1 << COM1A1) | (1 << COM1B1);
	TCCR1A |= (1 << WGM10);
	TCCR1B |= (1 << CS10) | (1 << CS11);
		//Bit 0 – TOIE1: Timer/Counter1, Overflow Interrupt Enable
		TIMSK1 = (0 << ICIE1) | (0 << OCIE1B) | (0 << OCIE1A) | (1 << TOIE1);
		//Enable interrupts globally
		sei();
	OCR1A = 0;
	OCR1B = 0;
}


void tim2_int(){
	DDRB |= (1 << DDB3);
	DDRD |= (1 << DDD3);
	//TCCR2A |= (1 << COM2A0) | (1 << COM2A1) | (1 << COM2B0) | (1 << COM2B1); //INVERTING MODE
	TCCR2A |= (1 << COM2A1) | (1 << COM2B1); // NON-INVERTING MODE
	TCCR2A |= (1 << WGM20);
	TCCR2B |= (1 << CS20) | (1 << CS21);
	OCR2A = 0;
	OCR2B = 0;
}

void PWM(unsigned char duty0A, unsigned char duty0B,unsigned char duty1A,unsigned char duty1B,unsigned char duty2A,unsigned char duty2B){

	OCR0A = pwm_table[duty0A];

	OCR0B = pwm_table[duty0B];

	OCR1A = pwm_table[duty1A];

	OCR1B = pwm_table[duty1B];

	OCR2A = pwm_table[duty2A];

	OCR2B = pwm_table[duty2B];
}

void delay_ms(int _speed)
{
	while (_speed--) {
		_delay_us(1000);  // one millisecond
	}
}

void kometa_vpred_ON(){
	while(vpred_ON == 1)
	{
		if (duty0A < 255)
		{duty0A++;
		}
		if (duty0A > 125 && duty0B < 255)
		{duty0B++;
		}
		if (duty0B > 125 && duty1A < 255)
		{duty1A++;
		}
		if (duty1A > 125 && duty1B < 255)
		{duty1B++;
		}
		if (duty1B > 125 && duty2A < 255)
		{duty2A++;
		}
		if (duty2A > 125 && duty2B < 255)
		{duty2B++;
		}
		if (duty2B == 255)
		{vpred_ON = 0;
		}
		PWM(duty0A, duty0B, duty1A , duty1B, duty2A, duty2B);
		
		_delay_us(500);
	}
}

void kometa_vpred_OFF(){
	while(vpred_OFF == 1)
	{
		if (duty0A > 0)
		{duty0A--;
		}
		if (duty0A <= 125 && duty0B > 0)
		{duty0B--;
		}
		if (duty0B <= 125 && duty1A > 0)
		{duty1A--;
		}
		if (duty1A <= 125 && duty1B > 0)
		{duty1B--;
		}
		if (duty1B <= 125 && duty2A > 0)
		{duty2A--;
		}
		if (duty2A <= 125 && duty2B > 0)
		{duty2B--;
		}
		if (duty2B == 0)
		{vpred_OFF = 0;
		}
		PWM(duty0A, duty0B, duty1A , duty1B, duty2A, duty2B);
		_delay_us(500);
	}
}

void kometa_odzadu_ON(){
	while(dozadu_ON == 1)
	{
		if (duty2B < 255)
		{duty2B++;
		}
		if (duty2B > 125 && duty2A < 255)
		{duty2A++;
		}
		if (duty2A > 125 && duty1B < 255)
		{duty1B++;
		}
		if (duty1B > 125 && duty1A < 255)
		{duty1A++;
		}
		if (duty1A > 125 && duty0B < 255)
		{duty0B++;
		}
		if (duty0B > 125 && duty0A < 255)
		{duty0A++;
		}
		if (duty0A == 255)
		{dozadu_ON = 0;
		}
		PWM(duty0A, duty0B, duty1A , duty1B, duty2A, duty2B);
		_delay_us(500);
	}
}

void kometa_dozadu_OFF(){
	while(dozadu_OFF == 1)
	{
		if (duty2B > 0)
		{duty2B--;
		}
		if (duty2B <= 125 && duty2A > 0)
		{duty2A--;
		}
		if (duty2A <= 125 && duty1B > 0)
		{duty1B--;
		}
		if (duty1B <= 125 && duty1A > 0)
		{duty1A--;
		}
		if (duty1A <= 125 && duty0B > 0)
		{duty0B--;
		}
		if (duty0B <= 125 && duty0A > 0)
		{duty0A--;
		}
		if (duty0A == 0)
		{dozadu_OFF = 0;
		}
		PWM(duty0A, duty0B, duty1A , duty1B, duty2A, duty2B);
		_delay_us(500);
	}
}

void kometa_vlna(){
	while(vlna_ON == 1)
	{
//////////// PWM1 ZAP duty0A
		if (PWM1==0)
		{duty0A++;
			if (duty0A == 255)
			{PWM1=1;
			}
		} 
//////////// PWM2 ZAP duty0B					
		if (PWM2==0 && duty0A > 125)
		{duty0B++;
			if (duty0B == 255)
			{PWM2=1;
			}
		}
//////////// PWM3 ZAP duty1A		
		if (PWM3==0 && duty0B > 125)
		{duty1A++;
			if (duty1A == 255)
			{PWM3=1;
			}
		}
//////////// PWM4 ZAP duty1B
if (PWM4==0 && duty1A > 125)
{duty1B++;
	if (duty1B == 255)
	{PWM4=1;
	}
}
//////////// PWM5 ZAP duty2A
if (PWM5==0 && duty1B > 125)
{duty2A++;
	if (duty2A == 255)
	{PWM5=1;
	}
}
//////////// PWM6 ZAP duty2B
if (PWM6==0 && duty2A > 125)
{duty2B++;
	if (duty2B == 255)
	{PWM6=1;
	}
}
//////////// PWMXA ZAP
if (PWMXA==0 && duty2B > 125)
{XA++;
	if (XA == 255)
	{PWMXA=1;
	}
}
//////////// PWMXB ZAP
if (PWMXB==0 && XA > 125)
{XB++;
	if (XB == 255)
	{PWMXB=1;
	}
}
//////////// PWM1 VYP		
		if (PWM1==1 && duty1A > 125)
		{duty0A--;
			if (duty0A == 0)
			{PWM1=2;
			}
		}
//////////// PWM2 VYP
if (PWM2==1 && duty1B > 125)
{duty0B--;
	if (duty0B == 0)
	{PWM2=2;
	}
}
//////////// PWM3 VYP
if (PWM3==1 && duty2A > 125)
{duty1A--;
	if (duty1A == 0)
	{PWM3=2;
	}
}
/////////// PWM4 VYP
if (PWM4==1 && duty2B > 125)
{duty1B--;
	if (duty1B == 0)
	{PWM4=2;
	}
}
/////////// PWM5 VYP
if (PWM5==1 && XA > 125)
{duty2A--;
	if (duty2A == 0)
	{PWM5=2;
	}
}
/////////// PWM6 VYP
if (PWM6==1 && XB > 125)
{duty2B--;
	if (duty2B == 0)
	{PWM6=2;
	}
}
		if (PWM6==2)
		{vlna_ON = 0;
			XA=0;
			XB=0;
			PWM1 = 0;
			PWM2 = 0;
			PWM3 = 0;
			PWM4 = 0;
			PWM5 = 0;
			PWM6 = 0;
			PWMXA = 0;
			PWMXB = 0;
			//kom_ispeed = 0; // cca 6s
		}
		PWM(duty0A, duty0B, duty1A , duty1B, duty2A, duty2B);
		_delay_us(750);
	}
}

void kometa_trvale_ON(){
	while(kometa_ON == 1)
	{
		OCR0A = 255;
		OCR0B = 255;
		OCR1A = 255;
		OCR1B = 255;
		OCR2A = 255;
		OCR2B = 255;
		kometa_ON = 0;
	}
}

void kometa_trvale_OFF(){
	while(kometa_OFF == 1)
	{
		OCR0A = 0;
		OCR0B = 0;
		OCR1A = 0;
		OCR1B = 0;
		OCR2A = 0;
		OCR2B = 0;
		kometa_OFF = 0;
	}
}

void effekt(){
	
	if (kometa_count == kometa_speed[kom_ispeed])
	{
		cli();
	
		if (selection == 0 ) //0 - vlna
		{
			vpred_ON = 0;
			vpred_OFF = 0;
			dozadu_ON = 0;
			dozadu_OFF = 0;
			kometa_ON = 0;
			kometa_OFF = 0;
			kom_ispeed++;
			vlna_ON = 1;
		}
		
		if (selection == 1 ) //1 - efekt zapina sa vpred vypina sa vpred
		{
			switch (change)
			{
				case 'A':
				vpred_ON = 1;
				vpred_OFF = 0;
				dozadu_ON = 0;
				dozadu_OFF = 0;
				kometa_ON = 0;
				kometa_OFF = 0;
				vlna_ON = 0;
				change = 'B';
				kom_ispeed++;
				break;
				
				case 'B':
				vpred_ON = 0;
				vpred_OFF = 1;
				dozadu_ON = 0;
				dozadu_OFF = 0;
				kometa_ON = 0;
				kometa_OFF = 0;
				vlna_ON = 0;
				change = 'A';
				kom_ispeed++;
				break;
			}
		}
		
		if (selection == 2 ) //2 - strieda sa ekeft
		{
			switch (change)
			{
				case 'A':
				vpred_ON = 1;
				vpred_OFF = 0;
				dozadu_ON = 0;
				dozadu_OFF = 0;
				kometa_ON = 0;
				kometa_OFF = 0;
				vlna_ON = 0;
				change = 'B';
				kom_ispeed++;
				break;
				
				case 'B':
				vpred_ON = 0;
				vpred_OFF = 1;
				dozadu_ON = 0;
				dozadu_OFF = 0;
				kometa_ON = 0;
				kometa_OFF = 0;
				vlna_ON = 0;
				change = 'C';
				kom_ispeed++;
				break;

				case 'C':
				vpred_ON = 0;
				vpred_OFF = 0;
				dozadu_ON = 1;
				dozadu_OFF = 0;
				kometa_ON = 0;
				kometa_OFF = 0;
				vlna_ON = 0;
				change = 'D';
				kom_ispeed++;
				break;

				case 'D':
				vpred_ON = 0;
				vpred_OFF = 0;
				dozadu_ON = 0;
				dozadu_OFF = 1;
				kometa_ON = 0;
				kometa_OFF = 0;
				vlna_ON = 0;
				change = 'E';
				kom_ispeed++;
				break;
				
				case 'E':
				vpred_ON = 1;
				vpred_OFF = 0;
				dozadu_ON = 0;
				dozadu_OFF = 0;
				kometa_ON = 0;
				kometa_OFF = 0;
				vlna_ON = 0;
				change = 'F';
				kom_ispeed++;
				break;
				
				case 'F':
				vpred_ON = 0;
				vpred_OFF = 1;
				dozadu_ON = 0;
				dozadu_OFF = 0;
				kometa_ON = 0;
				kometa_OFF = 0;
				vlna_ON = 0;
				change = 'G';
				kom_ispeed++;
				break;
				
				case 'G':
				vpred_ON = 1;
				vpred_OFF = 0;
				dozadu_ON = 0;
				dozadu_OFF = 0;
				kometa_ON = 0;
				kometa_OFF = 0;
				vlna_ON = 0;
				change = 'H';
				kom_ispeed++;
				break;
				
				case 'H':
				vpred_ON = 0;
				vpred_OFF = 1;
				dozadu_ON = 0;
				dozadu_OFF = 0;
				kometa_ON = 0;
				kometa_OFF = 0;
				vlna_ON = 0;
				change = 'A';
				kom_ispeed++;
				break;
			}
		}
		
		if (selection == 3 ) //3 - trvale ON
		{
			vpred_ON = 0;
			vpred_OFF = 0;
			dozadu_ON = 0;
			dozadu_OFF = 0;
			kometa_ON = 1;
			kometa_OFF = 0;
			vlna_ON = 0;
		}

		if (selection == 4 ) //4 - trvale OFF
		{
			vpred_ON = 0;
			vpred_OFF = 0;
			dozadu_ON = 0;
			dozadu_OFF = 0;
			kometa_ON = 0;
			kometa_OFF = 1;
			vlna_ON = 0;
		}
		
		kometa_count = 0;
		if (kom_ispeed >= 100)
		{kom_ispeed = 0;
		}
		sei();
	}
}

void init(){
	vpred_ON = 0;
	vpred_OFF = 0;
	dozadu_ON = 0;
	dozadu_OFF = 0;
	kometa_ON = 0;
	kometa_OFF = 0;
	vlna_ON = 0;
	kom_ispeed = 0;
	kometa_count = 0;
	change = 'A';
	OCR0A = 0;
	OCR0B = 0;
	OCR1A = 0;
	OCR1B = 0;
	OCR2A = 0;
	OCR2B = 0;
	PWM1 = 0;
	PWM2 = 0;
	PWM3 = 0;
	PWM4 = 0;
	PWM5 = 0;
	PWM6 = 0;
	PWMXA = 0;
	PWMXB = 0;
	duty0A = 0;
	duty0B = 0;
	duty1A = 0;
	duty1B = 0;
	duty2A = 0;
	duty2B = 0;
}

int main(void)
{
	// https://sites.google.com/site/qeewiki/books/avr-guide/external-interrupts-on-the-atmega328
	DDRD &= ~(1 << DDD2);     // Clear the PD2 pin
	// PD2 (PCINT0 pin) is now an input

	PORTD |= (1 << PORTD2);    // turn On the Pull-up
	// PD2 is now an input with pull-up enabled

	//EICRA |= (1 << ISC00) | (1 << ISC01);    // The rising edge of INTx generates an interrupt request
	EICRA |= (1 << ISC01); // The falling edge of INTx generates an interrupt request
	EIMSK |= (1 << INT0);     // Turns on INT0
	
	sei();                    // turn on interrupts
	
	tim0_int();
	tim1_int();
	tim2_int();
	change = 'A';
	selection = 0;
	
	while (1)
	{
		effekt();
		kometa_vpred_ON();
		kometa_vpred_OFF();
		kometa_odzadu_ON();
		kometa_dozadu_OFF();
		kometa_trvale_ON();
		kometa_trvale_OFF();
		kometa_vlna();
	}
}

ISR(TIMER1_OVF_vect){
		
		OVF_update++;
		
		if (OVF_update >= 30)
		{
			kometa_count++;
					
			OVF_update = 0;
		}
}


ISR (INT0_vect)
{
	init();
	selection++;
	if(selection==5)
	selection =0;
	/// zablokovanie efektu 2
	if(selection==2)
	selection =3;
	//0 - vlna
	//1 - efekt zapina sa vpred vypina sa vpred
	//2 - strieda sa ekeft
	//3 - trvale ON
	//4 - trvale OFF
}