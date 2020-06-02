#ifndef F_CPU
#define F_CPU 1000000UL
#endif

#ifndef BAUD
#define BAUD 9600UL
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <util/setbaud.h>
#include <avr/interrupt.h>

typedef union
{
	struct {
		char direction;
		uint16_t speed;
	} __attribute__((__packed__));
} Instruction;


uint16_t lastButtonTime = 0;
uint16_t lastTransmitTime = 0;
uint16_t lastLoopTime;
uint16_t newLoopTime;
uint16_t lastADC = 0;
volatile uint16_t overflow = 0;
volatile uint8_t canPressButton = 1;
volatile Instruction currentData;


#define TRANSMIT_TIME 400
#define WAIT_TIME 180


//This interrupt is called whenever the forwards, reverse or stop button is pressed
//It uses WAIT_TIME to debounce the button, which was especially important because
//the buttons I used had a lot of bounce - I got this value by trial and error
ISR(PCINT0_vect){

	if(canPressButton == 1){
		canPressButton = 0;
		currentData.speed = lastADC;
		if(bit_is_clear(PINB, 1)){
			currentData.direction = 'f';		
		} else if(bit_is_clear(PINB, 2)){
			currentData.direction = 'b';		
		} else {
			currentData.speed = 0;
			currentData.direction = 's';		
		}
	}	
}

ISR(TIMER0_OVF_vect){
	overflow++;
}

void initInterrupts(){
    PCICR |= (1 << PCIE0);
    PCMSK0 |= (1 << PCINT1) | (1 << PCINT2) | (1 << PCINT3);
	TIMSK0 = (1 << TOIE0);
    sei();
}

void initPWMAndTimers(){
    TCCR0B = (1 << CS02) | (1 << CS00); //1 tick is 1 ms
}

void initSerial(){
    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;

    UCSR0A = (1 << U2X0);
    UCSR0B = (1 << RXEN0) | (1 << RXCIE0) | (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void initADC(){
	//Initialize ADC0 in free running mode
	ADMUX |= (1 << REFS0);
	ADCSRA = (1 << ADPS1) | (1 << ADPS0);
	ADCSRA |= (1 << ADEN);
	ADCSRA |= (1 << ADATE);
	ADCSRA |= (1 << ADSC);
}

void transmit(volatile Instruction *toTransmit){
	cli();

	//Converts the struct to bytes to transmit them via serial
	uint8_t * temp = (uint8_t*)&currentData; 

	for(int i = 0; i < 3; i++){

		//Sends each byte individually
		while(!(UCSR0A & (1 << UDRE0))){
		}

		
		UDR0 = *(temp + i);
		//This delay seems to reduce issues I had with dropped bytes
		_delay_ms(3); 
	}
	sei();
}

int main(void){
    initSerial();
    initPWMAndTimers();
    initInterrupts();
    initADC();

    DDRB = 0b00000000;
    PORTB = 0b00001110;

	currentData.speed = 0;
	currentData.direction = 's';

	lastLoopTime = TCNT0;

    while(1){
		
		cli();
		lastADC = ADC / 4; //Scale it between 0 and 256
		sei();


		newLoopTime = TCNT0;		
		uint16_t delta = ((255UL * overflow) + newLoopTime) - lastLoopTime;
		overflow = 0;
		lastLoopTime = newLoopTime;
    	
		//Increment the counters by the delta time since the last iteration
		//of the main loop
		lastButtonTime += delta;
		lastTransmitTime += delta;

		if(lastButtonTime >= WAIT_TIME){
			lastButtonTime = 0;
			canPressButton = 1;
		}

		if(lastTransmitTime >= TRANSMIT_TIME){
			//This automatically transmits at every TRANSMIT_TIME ms
			lastTransmitTime = 0;
			transmit(&currentData);
		}

		_delay_ms(1);
	}
   
    return 0;
}

