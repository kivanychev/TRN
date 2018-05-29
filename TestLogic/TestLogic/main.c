/*
 * TestLogic.c
 *
 * Created: 24.05.2018 19:00:55
 * MCU: ATMEGA328P
 * Author : Kirill Ivanychev
 */ 

/************************************************************************/
/* INCLUDES                                                             */
/************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>

/************************************************************************/
/* CONSTANTS                                                            */
/************************************************************************/


/************************************************************************/
/* MACROS                                                               */
/************************************************************************/

#define StartConvAdc() ADCSRA |= (1<<ADSC)

/************************************************************************/
/* LOCAL VARIABLES                                                      */
/************************************************************************/



/************************************************************************/
/* IMPLEMENTATION                                                       */
/************************************************************************/

int main(void)
{
    // Initializing IO ports
    DDRB = 0x0f;
    PORTB = 0xff;

    /**********************************/
    /* Initializing ADC:              */
    /* AREF to Vcc, set AD0 channel   */
    /**********************************/
    
    ADMUX = (0<<REFS1) | (1<<REFS0);

    // Turn on ADC, Single conversion mode, Enable ADC interrupts
    // Set conversion frequency to FCPU/128
    ADCSRA = (1<<ADEN) | (1<<ADSC) | (1<<ADIE) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);

    /*********************************/
    /* Initialize Timer1             */
    /*                               */
    /* Clock prescaler = 1024        */
    /* OC mode: clear on compare     */
    /* PWM: 10 bit, A, B, WGM = 0111 */
    /*********************************/

    TCCR1A = (1 << COM1B1) + (1 << COM1A1) + (1 << WGM10) + (1 << WGM11);
    TCCR1B = (1 << WGM12) + (1 << CS12) + (1 << CS10);
    TCCR1C = 0;

    OCR1A = ZERO_DIFF;    // Channel A: set difference = 0
    OCR1B = ZERO_DIFF;    // Channel B: set difference = 0


    sei();
    StartConvAdc();

    // Checking pressed buttons
    while(1)
    {
    }            
}

/************************************************************************/
/* ADC Interrupt handler                                                */
/*  Reads ADC data and transfers it to PWM                              */
/*                                                                      */
/************************************************************************/

ISR(ADC_vect)
{
    unsigned char AdcBuf = ADCH;
    
    // Restarting AD conversion before exit
    StartConvAdc();
}
