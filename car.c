#ifndef F_CPU
#define F_CPU 1000000UL
#endif

#ifndef BAUD
#define BAUD 9600UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/setbaud.h>
#include <util/delay.h>
#include<stdlib.h>

//Play with these 2 values a bit to determine how responsive the car is
#define HYSTERISIS 8L
#define ACCELERATION 6UL

typedef union
{
	struct __attribute__((__packed__)){
		char direction;
		uint16_t speed;
	};
} Instruction;

/* This determines how the car is currently moving - note that this is
relative to the current direction, so going from reversing at a lower speed
to reversing at a higher speed is accelerating, as is going forwards moving from a
lower speed to a higher speed. FLIPDIRECTION is a special case where you move from
forwards to reverse or vice versa */

typedef enum {STEADY, ACCELERATING, DECELERATING, FLIPDIRECTION} MovementMode;

volatile uint8_t myChar;
volatile uint8_t overflow;
volatile uint8_t hasNewData = 0;
volatile uint8_t tempBuffer[3];
volatile uint8_t dataCount;
Instruction currentData;

uint16_t lastTime;
uint16_t targetSpeed;
uint16_t tempTargetSpeed;
char tempTargetDirection = 's';
uint16_t currentSpeed = 0;
char currentDirection = 's';
MovementMode currentMode = STEADY;

void initTimer(){
    TCCR1B = (1 << CS10) |(1 << WGM13) | (1 << WGM11);
    TCCR1A = (1 << COM1A1);
    ICR1 = 256;
    OCR1A = 0;
    TCCR0B = (1 << CS12) | (1 << CS10);
    TIMSK0 = (1 << TOIE0);
}

void initSerial(){
    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;

    UCSR0A = (1 << U2X0);
    UCSR0B = (1 << RXEN0) | (1 << RXCIE0) | (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

long clamp(long val, long min, long max){
    if(val >= max){
        return max;
    } else if(val <= min){
        return min;
    } else{
        return val;
    }
}

//Modifies the ports to say if the car is going forwards or backwards
void applyDirection(){
    if(currentDirection == 'f'){
        PORTB = (1 << PB3);
    } else if(currentDirection == 'b'){
        PORTB = (1 << PB2);
    } else{
        PORTB = 0;
    }
}

//Calculates what speed to output via PWM through OC1A
//by factoring the acceleration with the delta time
void applySpeed(uint16_t delta){
    if(currentMode == ACCELERATING){
        long tempCS = currentSpeed + (ACCELERATION * delta);
        currentSpeed = clamp(tempCS, 0, targetSpeed);
        OCR1A = currentSpeed;
        applyDirection();
        if(currentSpeed == targetSpeed){
            currentMode = STEADY;
        }
    } else if(currentMode == DECELERATING){
        long tempCS = currentSpeed - (ACCELERATION * delta);
        currentSpeed = clamp(tempCS, targetSpeed, 255);
        OCR1A = currentSpeed;
        applyDirection();
        if(currentSpeed == targetSpeed){
            currentMode = STEADY;
            if(currentSpeed == 0){
                currentDirection = 's'; //The vehicle has decelerated to a halt
            }
        }
    } else if(currentMode == STEADY){
        OCR1A = currentSpeed;
        applyDirection();
    } else {
        //Flipping direction has a deceleration phase followed by an acceleration phase
        long tempCS = currentSpeed - (ACCELERATION * delta);
        currentSpeed = clamp(tempCS, 0, 255);
        OCR1A = currentSpeed;
        applyDirection();
        if(currentSpeed == 0){
            currentDirection = tempTargetDirection;
            targetSpeed = tempTargetSpeed;
            currentMode = ACCELERATING;
            applyDirection();
        }
    }

}

int main(void){
    //Set up our ports as outputs
    DDRB = (1 << PB1) | (1 << PB2) | (1 << PB3);

    //Turn on our serial and timers and enable interrupts
    initSerial();
    initTimer();
    sei();

    lastTime = TCNT0;

    while(1){

        /*The delta is calculated as currentTime - lastTime
        However, since we're using timer 0 at 1 ms per tick, we can
        only get 256 ms out of our timer before it overflows. Incrementing an overflow
        flag in the timer overflow interrupt and multiply it by 255 then adding it to
        the current time, gives us the actual currentTime since the last loop.

        It's possible that the loop is short enough to avoid overflows, but I never
        timed it, and this was simple enough to implement that I didn't need to. */
        uint16_t currentTime = TCNT0;
        uint16_t delta = ((255UL * (uint16_t)overflow) + currentTime) - lastTime;
        if (hasNewData == 1){

            /* Some ugly pointer arithmetic which converts our array of 3 uint8_ts
            to an Instruction datatype - it creates an Instruction pointer to the start
            of the buffer array, then deferences the pointer to get the actual value. A bit
            of a hack, but it does what it's supposed to.  */
            currentData = *((Instruction*)tempBuffer);
            hasNewData = 0;

            if(currentDirection == currentData.direction){
                //In this clause, there has been no change in direction, just speed,
                //so we need to calculate the relevant speed from the acceleration to get there.
                if(currentDirection == 's'){
                    currentSpeed = 0;
                    targetSpeed = 0;
                    currentMode = STEADY;
                } else {
                    if((int)currentSpeed >= (int)(currentData.speed) + HYSTERISIS){
                        targetSpeed = currentData.speed;
                        currentMode = DECELERATING;
                    } else if((int)currentSpeed <= (int)(currentData.speed) - HYSTERISIS){
                        targetSpeed = currentData.speed;
                        currentMode = ACCELERATING;
                    } else{
                        currentSpeed = currentData.speed;
                        targetSpeed = currentSpeed;
                        currentMode = STEADY;
                    }
                }
            } else if(currentDirection == 's' && currentData.direction != 's'){
                //This clause means that we move from rest to either forwards or backwards
                currentSpeed = 0; //Just to be safe
                targetSpeed = currentData.speed;
                currentMode = ACCELERATING;
                currentDirection = currentData.direction;
            } else if(currentDirection != 's' && currentData.direction == 's'){
                //This clause means we're moving and we get a command to stop
                targetSpeed = 0;
                currentMode = DECELERATING;
            } else{
                //This means we're switching directions - either going
                //from forwards to backwards, or vice-versa
                tempTargetSpeed = currentData.speed;
                tempTargetDirection = currentData.direction;
                currentMode = FLIPDIRECTION;                
            }
        }

        lastTime = currentTime;
        overflow = 0;
        applySpeed(delta);
          
    }

    return 0;
}

ISR(USART_RX_vect){
    /* Read the byte off the buffer, add it to our temporary buffer
    and when we have 3 bytes, let the main loop know that we have a new Instruction
    to be converted from bytes */
    myChar = UDR0;
    tempBuffer[dataCount] = myChar;
    dataCount += 1;
    if(dataCount == 3){
        dataCount = 0;
        hasNewData = 1;
    }
}

ISR(TIMER0_OVF_vect){
    overflow++;	
}
