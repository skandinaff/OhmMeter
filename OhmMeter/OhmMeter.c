/*
 * _7SegDispMega2560.c
 *
 * Created: 06.05.2015 14:19:27
 *  Author: Aleksandrs
 */ 

#define F_CPU 16000000

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define SegDataPORT PORTA
#define SegDataDDR	DDRA

#define SegComPORT PORTC
#define SegComDDR  DDRC

#define SegComPin1 PC0
#define SegComPin2 PC2
#define SegComPin3 PC4
#define SegComPins (1<<SegComPin1)|(1<<SegComPin2)|(1<<SegComPin3)

#define LedDDR DDRG
#define LedPORT PORTG
#define LED_GREEN PG2
#define LED_RED PG0

#define PRESC 64

#define RANGE_PIN PINL

#define VIN 5.0
#define R_REFERENCE 474.0

char seg_buf[3];
	
char tab_seg[10] = {0x3F, 0x06, 0x5B, 0x4F, 0x66,
0x6D, 0x7D, 0x07, 0x7F, 0x6F};

volatile char digit;
volatile int limit;

 ISR(TIMER0_COMPA_vect)
 {

	digit++;
	if (digit >2) digit = 0;

 	SegComPORT = SegComPins; // Turn off all digits

	switch(digit)
	{
		case 0:
			SegComPORT &= ~(1<<SegComPin3); // 0b00000101;
			break;
		case 1:
			SegComPORT &= ~(1<<SegComPin2); //0b00010001;
			break;
		case 2:
			SegComPORT &= ~(1<<SegComPin1); // 0b00010100;
			break;
	}
	
 	SegDataPORT = seg_buf[digit]; // Lit current digit

 }

 void init_timer0()
 {
 	OCR0A = F_CPU/PRESC/1000 - 1;     //249, supposedly 1ms long interrupt
 	
 	TCCR0A |= (1<<WGM01);			//Set to CTC mode
 	TCCR0B |= (1<<CS00)|(1<<CS01);  //Prescaler of 64
	//if F_CPU = 1M => prescaler = 8
 	TIMSK0 |= (1<<OCIE0A);
 	
 	sei();
 	
 }

void init_ADC()
{
	ADMUX |= (1<<REFS0); 
	ADCSRA |= (1<<ADEN); // Enable ADC
	ADCSRA |= (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); // Prescaler of 128
	// If F_CPU = 1M => prescaler = 8
}

uint16_t read_ADC()
{

	ADCSRA |= (1<<ADSC);
	
	return(ADC);
	
}

void printbuf(int value)        // lookup segments and put in buffer
{
	char pos;
	
	for (pos = 0; pos < 3; pos++) {
				
		seg_buf[pos] = tab_seg[value % 10];
		value /= 10;
	}
	
}

float calc_resist(uint16_t raw)
{
	float Vin = VIN;
	float Vout = 0.0;
	float R = R_REFERENCE;
	float Rx = 0.0;
	float buffer = 0.0;
	
	buffer = raw * Vin;
	Vout = buffer/1024.0;
	buffer = (Vin/Vout)-1;
	Rx = R*buffer;
	
	if(Rx > 999.0) Rx = 999;
	
	return(Rx);
}



void init_7Seg()
{
	SegDataPORT = 0xff;
	SegDataDDR = 0xff;
	
	SegComDDR = SegComPins;
	SegComPORT = SegComPins;
}

void test_limits(int R)
{

		if(R < limit && R>0){
			LedPORT |= (1<<LED_GREEN);	
			LedPORT &= ~(1<<LED_RED);
		}
		if(R >= limit ){
			LedPORT |= (1<<LED_RED);
			LedPORT &= ~(1<<LED_GREEN);
		}
		if(R==0) {
		LedPORT &= ~(1<<LED_RED);
		LedPORT &= ~(1<<LED_GREEN);
		}		
		
	
}

void range_select()
{
	//DDRL |= (0<<PL1)|(0<<PL3)|(0<<PL5);
	if(RANGE_PIN==0) limit = 0x0032; //50;  
	if(RANGE_PIN==1) limit = 0x0064; //100;
	if(RANGE_PIN==2) limit = 0x0096; //150;
	if(RANGE_PIN==3) limit = 0x012C; //300;
	if(RANGE_PIN==4) limit = 0x01F4; //500;
	if(RANGE_PIN==5) limit = 0x0320; //800;
	if(RANGE_PIN==6) limit = 0x0014; //20;
	if(RANGE_PIN==7) limit = 0x001E; //30;	
}

void init_led()
{
	LedDDR |= (1<<LED_RED)|(1<<LED_GREEN);
	LedPORT &= ~(1<<LED_RED)|(1<<LED_GREEN);
}

int main(void)
{
	
	init_timer0();
	init_ADC();	
	init_7Seg();
	init_led();
	
	range_select();
	
	while(1)
	{
		int r = 0;
		if(read_ADC()) r = calc_resist(read_ADC());
		
		test_limits(r);
		
		printbuf(r);
        _delay_ms(150);

	}
}