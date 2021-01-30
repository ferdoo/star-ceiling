/*******************************************************************
MCU:	ATmega328P
 
Timer/Counter0 has 2 outputs, OC0A and OC0B
Timer/Counter1 has 2 outputs, OC1A and OC1B
Timer/Counter2 has 2 outputs, OC2A and OC2B

PIN1	PD3	OC2B (Timer/Counter2)
PIN9	PD5	OC0B (Timer/Counter0)
PIN10	PD6	OC0A (Timer/Counter0)

PIN13	PB1	OC1A (Timer/Counter1)
PIN14	PB2	OC1B (Timer/Counter1)
PIN15	PB3	OC2A (Timer/Counter2)

TCCR0A	Timer/Counter Control Register 0 A
WGM		Waveform Generator Mode bits
CS		Clock Select (prescaler - preddelicka)

*******************************************************************
1. definovanie F_CPU pre vsetky include - aby rychlost platila aj na vlozene funkcie
2. limity cez pole
3. rychlost zmeny cez pole - uz sa nevola jedna funkcia viac krat
4. moznost menit limity a rychlosti - ESTE UPRAVIT PODMIENKY?
5. osetrenie pretecenia - nezapisem do OCR menej ako 0 ani viac ako 255
6. limity a rychlosti cez dvojrozmerne pole
7. vyber limitov a rychlosti cez strukturu
8. doplnenie definovatelnej pauzi
9. doplnenie initializacie limitov
10. upravit moznost menit limity cela podmienka je zle ...!
11. zmena PWM na phase correct (pri fast PWM a zapise do OCRxx = 0 LED trochu svieti)
12. pridanie prerusenie pre timer1 - nastane pri preteceni casovaca, pouzije sa na oddialenie inkrementacie daneho PWM cim sa spomali
    docieli sa rozna rychlosn na vsetkych PWM kanaloch
13. pridal sa vstup INT0 - prerusenie na prepinanie efektov a zmena posledneho bitu pri padajucej hviezde z PD2 na PD7 kedze PD2 sa zacalo pouzivat ako vstup
14. pridanie vystupov padajucej hviezdy pod program prerusenia / oprava sw pri OCR = 255 led nesvietili ale preblikavali
15. pridanie pola s casom na aktualizaciu padajucej hviezdy
*******************************************************************/ 


#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

unsigned char change = 'A';
unsigned char timer1, ichange, intro_play, XA, XB, hviezda, selection = 0;
unsigned char PWM1, PWM2, PWM3, PWM4, PWM5, PWM6, PWMXA, PWMXB = 0; //smer
unsigned char duty0A, duty0B, duty1A, duty1B, duty2A, duty2B = 0; //hodnoty PWM

unsigned char OVF_update, OVF_update1, OVF_update2, OVF_update3, OVF_update4, OVF_update5, OVF_update6 = 0;
unsigned char PWM1_update, PWM2_update, PWM3_update, PWM4_update, PWM5_update, PWM6_update = 0;
unsigned int hviezda_change=0;
unsigned int hviezda_count = 0;

//							   --0A-- ,  --0B-- ,  --1A-- ,  --1B-- ,  --2A-- ,  --2B--  ,
//								0,   1,   2,   3,   4,   5,   6,   7,   8,   9,  10,  11,
//unsigned char limit[6][12]={ {200, 255,  90, 190,  10, 150,  30, 90,  10, 100,  90, 255},			//PWM okna 1 ( limit[0][0-11] )
				             //{50, 200,  254, 255,  10, 250,  30, 190,  30, 255,  10, 100},		//PWM okna 2 ( limit[1][0-11] )
				             //{50, 200,  30, 190,  254, 255,  30, 250,  10, 100,  30, 190},		//PWM okna 3 ( limit[2][0-11] )
				             //{50, 200,  30, 190,  10, 250,  254, 255,  10, 100,  30, 255},
							 //{30, 190,  30, 190,  10, 250,  30, 250,  254, 255,  30, 255},		//PWM okna 3 ( limit[2][0-11] )
							 //{10, 250,  30, 190,  50, 200,  30, 250,  10, 100,  254, 255} };		//PWM okna 4 ( limit[3][0-11] )

unsigned char limit[6][12]={
{ 155, 255, 105, 255, 205, 255, 155, 255, 115, 255, 205, 255},
{ 105, 255, 155, 255, 115, 255, 205, 255, 205, 255, 115, 255},
{ 155, 255, 205, 255, 155, 255, 115, 255, 155, 255, 205, 255},
{ 105, 255, 55, 255, 45, 255, 35, 255, 25, 255, 15, 255},
{ 155, 255, 1, 255, 155, 255, 165, 255, 175, 255, 185, 255},
{ 95, 255, 85, 255, 205, 255, 195, 255, 155, 255, 155, 255},

};

//{254, 255,  120, 190,  90, 150,  90, 190,  10, 100,  155, 255},
//{100, 200,  254, 255,  100, 250,  90, 190,  120, 255,  100, 200},
//{50, 200,  100, 190,  254, 255,  90, 250,  90, 100,  90, 190},
//{50, 200,  70, 190,  70, 250,  254, 255,  10, 100,  90, 255},
//{30, 190,  90, 190,  90, 250,  90, 250,  254, 255,  90, 255},
//{90, 250,  120, 190,  150, 200,  90, 250,  10, 100,  254, 255}


//50, 200,  30, 190,  10, 250,  30, 250,  10, 100,  30, 255
//0, 255,  0, 255,  0, 255,  0, 255,  0, 255,  0, 255

//						   0A,0B,1A,1B,2A,2B
//							1, 2, 3, 4, 5, 6,
unsigned char update[6][6]={ {9, 6, 10, 6, 10, 8}, //0 - okamzite, 1 = 1x timer1 OVF ......
						   {7, 4, 7, 9, 8, 7},
						   {8, 7, 6, 9, 5, 12},
						   {6, 9, 4, 6, 7, 7},
						   {4, 6, 8, 5, 6, 4},
						   {10, 8, 6, 8, 7, 8} };

int _speed = 0;
int speed[9]= {5,3,4,6,7,5,3,4,5}; //hodnota v ms

							   
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
								 
struct tag_setting { unsigned char ilimit; //deklarace struktury
				     unsigned char iupdate;
					 unsigned char ispeed;
					 unsigned char ispeedHV; };
struct tag_setting set = {0, 0, 0, 0 }; // definice strukturní premennej
	
// program padajucej hviezdy (pauzy medzi ledkami)
unsigned int hviezda_speed[18][11]={ 
{ 10, 30, 50, 70, 90, 110, 130, 150, 170, 190, 210},
{ 10, 30, 50, 70, 120, 170, 220, 270, 290, 310, 330},
{ 10, 30, 50, 70, 90, 110, 130, 150, 170, 190, 210},
{ 10, 30, 50, 120, 190, 260, 280, 300, 320, 340, 360},
{ 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110},
{ 10, 30, 50, 70, 90, 110, 130, 150, 170, 190, 210},
{ 10, 30, 50, 80, 120, 170, 230, 280, 320, 350, 370},
{ 10, 30, 50, 70, 90, 110, 130, 180, 230, 280, 330},
{ 10, 30, 60, 110, 180, 270, 340, 390, 420, 440, 460},
{ 10, 30, 50, 70, 90, 110, 130, 150, 170, 190, 210},
{ 10, 30, 50, 70, 90, 130, 170, 210, 230, 250, 270},
{ 10, 30, 50, 70, 90, 110, 130, 150, 170, 190, 210},
{ 10, 30, 50, 80, 120, 170, 220, 260, 290, 310, 330},
{ 10, 30, 50, 70, 90, 110, 130, 150, 170, 190, 210},
{ 10, 30, 50, 70, 90, 120, 160, 210, 260, 310, 330},
{ 10, 30, 50, 70, 90, 110, 130, 150, 170, 190, 210},
{ 10, 30, 50, 80, 120, 160, 190, 210, 230, 250, 270},
{ 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110}
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

void hviezda_int(){
	//Nastavit ako vystup
	DDRC |= (1 << DDC0) | (1 << DDC1) | (1 << DDC2) | (1 << DDC3) | (1 << DDC4) | (1 << DDC5);
	//vypnut - sinking 0V
	PORTC &= ~ (1 << PORTC0) | (1 << PORTC1) | (1 << PORTC2) | (1 << PORTC3) | (1 << PORTC4) | (1 << PORTC5);
	//Nastavit ako vystup
	DDRD |= (1 << DDD0) | (1 << DDD1) | (1 << DDD7);
	//vypnut - sinking 0V
	PORTD &= ~(1 << PORTD0) | (1 << PORTD1) | (1 << PORTD7);
	DDRB |= (1 << DDB0);
	//vypnut - sinking 0V
	PORTB &= ~(1 << PORTB0);
}

//*******************************************************************
//		inicializacia nastaveni
//*******************************************************************
void limit_ini(){
	
	set.ilimit = 0;
	set.iupdate = 0;
	set.ispeed = 0;
	
	duty0A = limit[set.ilimit][0];
	duty0B = limit[set.ilimit][2];
	duty1A = limit[set.ilimit][4];
	duty1B = limit[set.ilimit][6];
	duty2A = limit[set.ilimit][8];
	duty2B = limit[set.ilimit][10];
	
	PWM1_update=0;
	PWM2_update=0;
	PWM3_update=0;
	PWM4_update=0;
	PWM5_update=0;
	PWM6_update=0;

}

//*******************************************************************
//		Zapisanie hodnot do OCR (PWM) registra
//*******************************************************************
void PWM(unsigned char duty0A,unsigned char duty0B,unsigned char duty1A,unsigned char duty1B,unsigned char duty2A,unsigned char duty2B){
	if (duty0A >=0 && duty0A <= 255)
	{
		OCR0A = pwm_table[duty0A];
	}
	
	if (duty0B >=0 && duty0B <= 255)
	{
		OCR0B = pwm_table[duty0B];
	}
	
	if (duty1A >=0 && duty1A <= 255)
	{
		OCR1A = pwm_table[duty1A];
	}
	
	if (duty1B >=0 && duty1B <= 255)
	{
		OCR1B = pwm_table[duty1B];
	}
	
	if (duty2A >=0 && duty2A <= 255)
	{
		OCR2A = pwm_table[duty2A];
	}
	
	if (duty2B >=0 && duty2B <= 255)
	{
		OCR2B = pwm_table[duty0B];
	}
	
}

//*******************************************************************
//		Inkrementacia PWM 1 - 6
//*******************************************************************
void change_PWM(){
	if(update[set.iupdate][0] == 0 || PWM1_update == 1){
		if(PWM1==0)
		{duty0A++;
			if(duty0A >= limit[set.ilimit][1]){PWM1=1;}
			
		}
		else{
			duty0A--;
			if(duty0A <= limit[set.ilimit][0]){PWM1=0;}
		}
		PWM1_update=0;
	}
	
	if(update[set.iupdate][1] == 0 || PWM2_update == 1){
		if(PWM2==0)
		{duty0B++;
			if(duty0B >= limit[set.ilimit][3]){PWM2=1;}
		}
		else{
			duty0B--;
			if(duty0B <= limit[set.ilimit][2]){PWM2=0;}
		}
		PWM2_update=0;
	}
	
	if(update[set.iupdate][2] == 0 || PWM3_update == 1){
		if(PWM3==0)
		{duty1A++;
			if(duty1A >= limit[set.ilimit][5]){PWM3=1;}
		}
		else{
			duty1A--;
			if(duty1A <= limit[set.ilimit][4]){PWM3=0;}
		}
		PWM3_update=0;
	}
	
	if(update[set.iupdate][3] == 0 || PWM4_update == 1){
		if (PWM4==0)
		{duty1B++;
			if (duty1B >= limit[set.ilimit][7])
			{PWM4=1;
			}
		}
		else
		{duty1B--;
			if (duty1B <= limit[set.ilimit][6])
			{PWM4=0;
			}
		}
		PWM4_update=0;
	}
	
	if(update[set.iupdate][4] == 0 || PWM5_update == 1){
		if (PWM5==0)
		{duty2A++;
			if (duty2A >= limit[set.ilimit][9])
			{PWM5=1;
			}
		}
		else
		{duty2A--;
			if (duty2A <= limit[set.ilimit][8])
			{PWM5=0;
			}
		}
		PWM5_update=0;
	}
	
	if(update[set.iupdate][5] == 0 || PWM6_update == 1){
		if (PWM6==0)
		{duty2B++;
			if (duty2B >= limit[set.ilimit][11])
			{PWM6=1;
			}
		}
		else
		{duty2B--;
			if (duty2B <= limit[set.ilimit][10])
			{PWM6=0;
			}
		}
		PWM6_update=0;
	}
}

//*******************************************************************
//		pauza
//*******************************************************************
void delay_ms(int _speed)
{
	while (_speed--) {
		_delay_us(1000);  // one millisecond
	}
}

//*******************************************************************
//		Zmena okna hranice PWM
//*******************************************************************
void hranice_PWM(){
	
	if (OVF_update == 255)
	{timer1++;
	}
	
	if (timer1 == 40 )
	{
		
		switch (change){
			case 'A':
			set.ilimit = 0;
			set.iupdate = 0;
			set.ispeed = 0 + ichange;
			change = 'B';
			break;
			
			case 'B':
			set.ilimit = 1;
			set.iupdate = 1;
			set.ispeed = 0 + ichange;
			change = 'C';
			break;

			case 'C':
			set.ilimit = 2;
			set.iupdate = 2;
			set.ispeed = 0 + ichange;
			change = 'D';
			break;

			case 'D':
			set.ilimit = 3;
			set.iupdate = 3;
			set.ispeed = 0 + ichange;
			change = 'E';
			break;
			
			case 'E':
			set.ilimit = 4;
			set.iupdate = 4;
			set.ispeed = 0 + ichange;
			change = 'F';
			break;
			
			case 'F':
			set.ilimit = 5;
			set.iupdate = 5;
			set.ispeed = 0 + ichange;
			change = 'A';
			ichange++;
			if (ichange == 9)
			{ichange = 0;
			}
						
			break;
			
		}
		
		timer1 = 0;
	}
}

//*******************************************************************
//		prve spustenie - vlna
//*******************************************************************
void intro(){
	while(intro_play == 1){
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
		{intro_play = 0;
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
		}
		PWM(duty0A, duty0B, duty1A , duty1B, duty2A, duty2B);
		_delay_us(750);
	}
}


void init_OCR(){

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

void init_OUT(){

	PORTC = 0;
	PORTD = 0;
	PORTB = 0;
}

void obloha_ON(){
	OCR0A = 255;
	OCR0B = 255;
	OCR1A = 255;
	OCR1B = 255;
	OCR2A = 255;
	OCR2B = 255;
}

//*******************************************************************
//		Main program
//*******************************************************************
int main(void)
{
		
		// https://sites.google.com/site/qeewiki/books/avr-guide/external-interrupts-on-the-atmega328
		DDRD &= ~(1 << DDD2);     // Clear the PD2 pin
		// PD2 (PCINT0 pin) is now an input

		PORTD |= (1 << PORTD2);    // turn On the Pull-up
		// PD2 is now an input with pull-up enabled

		//EICRA |= (1 << ISC00) | (1 << ISC01);    // The rising edge of INTx generates an interrupt request
		EICRA |= (1 << ISC01);
		EIMSK |= (1 << INT0);     // Turns on INT0
			
		sei();                    // turn on interrupts
		
		tim0_int();
		tim1_int();
		tim2_int();
		hviezda_int();
		intro_play = 1;
		hviezda = 1;
		intro();
		limit_ini();
			
		while (1)
		{
			
			if (selection == 0 || selection == 1) //hviezdna obloha ON
			{change_PWM();
			} 
			else
			{
				//hviezdna obloha OFF init ak je na OCR cislo rozne od 0 alebo ak nieje TRVALO ON
				if (OCR0A != 0 || OCR0B != 0 || OCR1A != 0 || OCR1B != 0 || OCR2A != 0 || OCR2B != 0 || selection == 4 )
				{init_OCR();
				}
			}
			
			
			if (selection == 4) // //hviezdna obloha TRVALE ON
			{
				obloha_ON();
			}
			else
			{
				PWM(duty0A, duty0B, duty1A , duty1B, duty2A, duty2B);
				hranice_PWM();
			}
			
			
			

			delay_ms(_speed);
			_speed = speed[set.ispeed];
		}
}

//*******************************************************************
//		Prerusenie
//*******************************************************************
ISR(TIMER1_OVF_vect){

	OVF_update++;
	if (hviezda_change >= hviezda_speed[set.ispeedHV][10]) //MAX cas cakania - LED10
	{hviezda_count++;
	}
	if (hviezda_change < hviezda_speed[set.ispeedHV][10]) //MAX cas cakania - LED10
	{hviezda_change++;
	}
		
	if (hviezda_count == (hviezda_speed[set.ispeedHV][9] * hviezda_speed[set.ispeedHV][10]) )
	{
		
			hviezda = 1;
			hviezda_count = 0;
			hviezda_change =0;
			
			if (set.ispeedHV < 17) // pocet programov v matici (17)
			{set.ispeedHV ++;
			} 
			else
			{set.ispeedHV = 0;
			}
				
	}
	
	
	if(hviezda == 1 && (selection == 0 || selection == 2)){
		if (hviezda_change == hviezda_speed[set.ispeedHV][0]) //LED1
		{
			PORTC = 1;
			PORTD = 0;
			PORTB = 0;
		}
		
		if (hviezda_change == hviezda_speed[set.ispeedHV][1]) //LED2
		{
			PORTC = 2;
			PORTD = 0;
			PORTB = 0;
		}
		
		if (hviezda_change == hviezda_speed[set.ispeedHV][2]) //LED3
		{
			PORTC = 4;
			PORTD = 0;
			PORTB = 0;
		}
		
		if (hviezda_change == hviezda_speed[set.ispeedHV][3]) //LED4
		{
			PORTC = 8;
			PORTD = 0;
			PORTB = 0;
		}
		
		if (hviezda_change == hviezda_speed[set.ispeedHV][4]) //LED5
		{
			PORTC = 16;
			PORTD = 0;
			PORTB = 0;
		}
		
		if (hviezda_change == hviezda_speed[set.ispeedHV][5]) //LED6
		{
			PORTC = 32;
			PORTD = 0;
			PORTB = 0;
		}
		
		if (hviezda_change == hviezda_speed[set.ispeedHV][6]) //LED7
		{
			PORTC = 0;
			PORTD = 0;
			PORTB = 1;
		}
		
		if (hviezda_change == hviezda_speed[set.ispeedHV][7]) //LED8
		{
			PORTC = 0;
			PORTD = 1;
			PORTB = 0;
		}
		
		if (hviezda_change == hviezda_speed[set.ispeedHV][8]) //LED9
		{
			PORTC = 0;
			PORTD = 2;
			PORTB = 0;
		}
		
		if (hviezda_change == hviezda_speed[set.ispeedHV][9]) //LED10
		{
			PORTC = 0;
			PORTD = 128;
			PORTB = 0;
		}
		
		if (hviezda_change == hviezda_speed[set.ispeedHV][10]) //LED ALL OFF
		{
			PORTC = 0;
			PORTD = 0;
			PORTB = 0;
			hviezda = 0;
			//hviezda_change=0;
		}
}			
else
{
	if (PORTC != 0 || PORTD != 0 ) // padajuca hviezda OFF init ak je na porte C alebo D nieco
	{init_OUT();
	}
}
	
	
	if (PWM1_update==0)
	{OVF_update1++;
	}
	if (PWM2_update==0)
	{OVF_update2++;
	}
	if (PWM3_update==0)
	{OVF_update3++;
	}
	if (PWM4_update==0)
	{OVF_update4++;
	}
	if (PWM5_update==0)
	{OVF_update5++;
	}
	if (PWM6_update==0)
	{OVF_update6++;
	}

	if (update[set.iupdate][0] == OVF_update1)
	{
		OVF_update1=0;
		PWM1_update=1;
	}
	if (update[set.iupdate][1] == OVF_update2)
	{
		OVF_update2=0;
		PWM2_update=1;
	}
	if (update[set.iupdate][2] == OVF_update3)
	{
		OVF_update3=0;
		PWM3_update=1;
	}
	if (update[set.iupdate][3] == OVF_update4)
	{
		OVF_update4=0;
		PWM4_update=1;
	}
	if (update[set.iupdate][4] == OVF_update5)
	{
		OVF_update5=0;
		PWM5_update=1;
	}
	if (update[set.iupdate][5] == OVF_update6)
	{
		OVF_update6=0;
		PWM6_update=1;
	}

}

ISR (INT0_vect)
{init_OCR();
	init_OUT();
	selection++;
	if(selection==5)
	selection = 0;
	//0 - hviezdna obloha ON - padajuca hviezda ON
	//1 - hviezdna obloha ON - padajuca hviezda OFF
	//2 - hviezdna obloha OFF - padajuca hviezda ON
	//3 - hviezdna obloha OFF - padajuca hviezda OFF
	//4 - hviezdna obloha TRVALE ON - padajuca hviezda OFF
	
}