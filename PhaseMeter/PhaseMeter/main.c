/*
 * PhaseMeter.c
 *
 * Created: 30.01.2018 8:06:19
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

// Timer period in mks
// Test: Check if timer interrupt is done every TIMER_PERIOD
#define TIMER_PERIOD    10  // us
#define MAX_ANGLE       90  // degrees

// Masks for channel pins
#define MASK_A 0b00000011
#define MASK_B 0b00001100
#define MASK_C 0b00110000

#define ZERO_DIFF   127

/************************************************************************/
/* LOCAL TYPES                                                          */
/************************************************************************/

/* Channel states types */
typedef enum {
    ST_START_A,
    ST_WAIT_A,
    ST_MEASURE_PLUS_A,
    ST_MEASURE_MINUS_A
    } TChannelStateA;

typedef enum {
    ST_START_B,
    ST_WAIT_B,
    ST_MEASURE_PLUS_B,
    ST_MEASURE_MINUS_B
} TChannelStateB;

typedef enum {
    ST_START_C,
    ST_WAIT_C,
    ST_MEASURE_PLUS_C,
    ST_MEASURE_MINUS_C
} TChannelStateC;


/* Pin states types */
typedef enum {
    ST_PIN_00_A = (0b00000000 << 0),
    ST_PIN_01_A = (0b00000001 << 0),
    ST_PIN_10_A = (0b00000010 << 0),
    ST_PIN_11_A = (0b00000011 << 0)
    } TPinStateA;

typedef enum {
    ST_PIN_00_B = (0b00000000 << 2),
    ST_PIN_01_B = (0b00000001 << 2),
    ST_PIN_10_B = (0b00000010 << 2),
    ST_PIN_11_B = (0b00000011 << 2)
} TPinStateB;

typedef enum {
    ST_PIN_00_C = (0b00000000 << 4),
    ST_PIN_01_C = (0b00000001 << 4),
    ST_PIN_10_C = (0b00000010 << 4),
    ST_PIN_11_C = (0b00000011 << 4)
} TPinStateC;


/************************************************************************/
/* LOCAL VARIABLES                                                      */
/************************************************************************/

TPinStateA curPinStateA;
TPinStateB curPinStateB;
TPinStateC curPinStateC;

TPinStateA nextPinStateA;
TPinStateB nextPinStateB;
TPinStateC nextPinStateC;

TChannelStateA channelStateA;
TChannelStateB channelStateB;
TChannelStateC channelStateC;

// Variables for measuring of the difference
volatile int diffA;
volatile int diffB;
volatile int diffC;

// Variables for fixed measured values of the differences
volatile int fixedDiffA;
volatile int fixedDiffB;
volatile int fixedDiffC;

volatile int powerPeriod;   // Sine Power Network time period in us
volatile int degreeTime;    // Time of 1' of network Sine (calculated)
volatile int maxDiffValue;  // Number of TIMER_PERIOD time intervals
volatile int maxDiffTime;   // us maximum difference time for [-MAX_ANGLE ... + MAX_ANGLE]

/************************************************************************/
/* IMPLEMENTATION                                                       */
/************************************************************************/

void Initialize()
{
    unsigned char temp;
    
    /* Initialize channel states */
    channelStateA = ST_START_A;
    channelStateB = ST_START_B;
    channelStateC = ST_START_C;

    curPinStateA = ST_PIN_00_A;
    curPinStateB = ST_PIN_00_B;
    curPinStateC = ST_PIN_00_C;

    nextPinStateA = ST_PIN_00_A;
    nextPinStateB = ST_PIN_00_B;
    nextPinStateC = ST_PIN_00_C;

    diffA = 0;
    diffB = 0;
    diffC = 0;
    
    /* Difference values that are ready for output calculations */
    fixedDiffA = 0;
    fixedDiffB = 0;
    fixedDiffC = 0;

    powerPeriod = 20000; // mks

    /* Enable A,B and C channel pins as inputs */
    DDRC = 0;
    
    /* Disable pull-ups */
    PORTC = 0;

    /* Enable toggle interrupts on PC0 -- PC7 (PCINT8..PCINT14) */
    PCICR = (1 << PCIE1);   
    PCMSK1 = (1 << PCINT8) | (1 <<PCINT9) | (1 <<PCINT10) | (1 <<PCINT11) | (1 <<PCINT12) | (1 <<PCINT13);
        
    /* Set output channel pins as outputs for dA, dB, dC */
    temp = DDRB;
    temp |= (1 << PORTB1) | (1 << PORTB2) | (1 << PORTB3);
    DDRB = temp;

    /********************************/
    /* Initialize Timer0            */
    /* Measures time                */
    /* Clock prescaler CKL/8        */
    /* Operation mode: CTC: WGM=010 */
    /* Interrupt on Compare Match   */
    /********************************/

    TCCR0A = (1 << WGM01);
    TCCR0B = (1 << CS01);
    TIMSK0 = (1 << OCIE0A);
    OCR0A = TIMER_PERIOD * 2; // us

    /********************************/
    /* Initialize Timer1            */
    /*                              */
    /* Clock prescaler = 1          */
    /* OC mode: clear on compare    */
    /* PWM: 8 bit, A, B, WGM = 0101 */
    /********************************/

    TCCR1A = (1 << COM1B1) + (1 << COM1A1) + (1 << WGM10);
    TCCR1B = (1 << WGM12) + (1 << CS10);
    TCCR1C = 0;

    OCR1A = ZERO_DIFF;    // Channel A: set difference = 0
    OCR1B = ZERO_DIFF;    // Channel B: set difference = 0

    /********************************/
    /* Initialize Timer2            */
    /*                              */
    /* Clock prescaler = 1          */
    /* OC mode: clear on compare    */
    /* PWM: 8 bit, A,  WGM = 011    */
    /********************************/
    
    TCCR2A = (1 << COM2A1) + (1 << WGM20) + (1 << WGM21);
    TCCR2B =  + (1 << CS20);
   
    OCR2A = ZERO_DIFF;    // Channel C: set difference = 0
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/

int main(void)
{
    Initialize();
    
    unsigned char outValueA;
    unsigned char outValueB;
    unsigned char outValueC;
    
    /********************************************/
    /* While power period is constant           */
    /* These 3 strings are out of while(1)      */
    /********************************************/    
    degreeTime = powerPeriod / 360;
    maxDiffTime = MAX_ANGLE * degreeTime;
    maxDiffValue = maxDiffTime / TIMER_PERIOD;

    sei();

    while (1)
    {        
        if(channelStateA == ST_START_A || channelStateA == ST_WAIT_A)
        {
            outValueA = ( ((long)(fixedDiffA + maxDiffValue)) << 8) / (maxDiffValue << 1);
            OCR1A = outValueA ;    // Channel A: set difference = 0
        }            

        if(channelStateB == ST_START_B || channelStateB == ST_WAIT_B)
        {
            outValueB = ( ((long)(fixedDiffB + maxDiffValue)) << 8) / (maxDiffValue << 1);
            OCR1B = outValueB;    // Channel B: set difference = 0
        }            

        if(channelStateC == ST_START_C || channelStateC == ST_WAIT_C)
        {
            outValueC = ( ((long)(fixedDiffC + maxDiffValue)) << 8) / (maxDiffValue << 1);
            OCR2A = outValueC;    // Channel C: set difference = 0
        }            
    }
}


/************************************************************************/
/* Timer 0 Compare match Interrupt handler                              */
/*                                                                      */
/* Measures time for each channel phase difference                      */
/*                                                                      */
/************************************************************************/

ISR(TIMER0_COMPA_vect)
{
    if(channelStateA == ST_MEASURE_PLUS_A)
    {
        diffA++;
    }        
    else if(channelStateA == ST_MEASURE_MINUS_A)
    {
        diffA--;
    }

    if(channelStateB == ST_MEASURE_PLUS_B)
    {
        diffB++;
    }        
    else if(channelStateB == ST_MEASURE_MINUS_B)
    {
        diffB--;
    }
    
    if(channelStateC == ST_MEASURE_PLUS_C)
    {
        diffC++;
    }        
    else if(channelStateC == ST_MEASURE_MINUS_C)
    {
        diffC--;
    }

}

/************************************************************************/
/* PIN Change Interrupt Handler                                         */
/* Handles all changes that appear on channel A, B and C                */
/* Measures time difference between start edges of the pulses on        */
/* each channel                                                         */
/************************************************************************/

ISR(PCINT1_vect)
{
    unsigned char pinState;
    
    pinState = PINC;
        
    nextPinStateA = (TPinStateA)(pinState & MASK_A);
    nextPinStateB = (TPinStateB)(pinState & MASK_B);
    nextPinStateC = (TPinStateC)(pinState & MASK_C);
    
    /**********************/
    /* Checking channel A */
    /**********************/
    switch(channelStateA)
    {   
        case ST_START_A:    
        /* Searching for initial state */
            if(nextPinStateA == ST_PIN_00_A)
            {
                channelStateA = ST_WAIT_A;
                curPinStateA = nextPinStateA;
            }
        break;
            
        case ST_WAIT_A:
            if(nextPinStateA != curPinStateA)
            {
                diffA = 0;
                fixedDiffA = 0;
                
                switch(nextPinStateA)
                {
                    case ST_PIN_01_A:
                        channelStateA = ST_MEASURE_MINUS_A;
                        break;
                            
                    case ST_PIN_10_A:
                        channelStateA = ST_MEASURE_PLUS_A;
                        break;
                            
                    case ST_PIN_11_A:
                    default:
                        channelStateA = ST_START_A;
                        break;

                }                        

                curPinStateA = nextPinStateA;
            }
            break;
            
        case ST_MEASURE_PLUS_A:
        case ST_MEASURE_MINUS_A:
            if(nextPinStateA != curPinStateA)
            {   /* Switching to next state */
                channelStateA = ST_START_A;

                if(nextPinStateA == ST_PIN_11_A)
                {
                    if(diffA > maxDiffValue)
                    {
                        diffA = maxDiffValue;
                    }

                    if(diffA < (-1* maxDiffValue))
                    {
                        diffA = (-1* maxDiffValue);
                    }
            
                    fixedDiffA = diffA;
                }
                else
                {
                    diffA = 0;
                    fixedDiffA = 0;
                }                        

                curPinStateA = nextPinStateA;
            }
            break;
            
        default:
            break;
        
    }

    /**********************/
    /* Checking channel C */
    /**********************/
    switch(channelStateC)
    {
        case ST_START_C:
        /* Searching for initial state */
        if(nextPinStateC == ST_PIN_00_C)
        {
            channelStateC = ST_WAIT_C;
            curPinStateC = nextPinStateC;
        }
        break;
        
        case ST_WAIT_C:
        if(nextPinStateC != curPinStateC)
        {
            diffC = 0;
            fixedDiffC = 0;
            
            switch(nextPinStateC)
            {
                case ST_PIN_01_C:
                channelStateC = ST_MEASURE_MINUS_C;
                break;
                
                case ST_PIN_10_C:
                channelStateC = ST_MEASURE_PLUS_C;
                break;
                
                case ST_PIN_11_C:
                default:
                channelStateC = ST_START_C;
                break;

            }

            curPinStateC = nextPinStateC;
        }
        break;
        
        case ST_MEASURE_PLUS_C:
        case ST_MEASURE_MINUS_C:
        if(nextPinStateC != curPinStateC)
        {   /* Switching to next state */
            channelStateC = ST_START_C;

            if(nextPinStateC == ST_PIN_11_C)
            {
                if(diffC > maxDiffValue)
                {
                    diffC = maxDiffValue;
                }
            
                if(diffC < (-1* maxDiffValue))
                {
                    diffC = (-1* maxDiffValue);
                }
                
                fixedDiffC = diffC;
            }
            else
            {
                diffC = 0;
                fixedDiffC = 0;
            }

            curPinStateC = nextPinStateC;
        }
        break;
        
        default:
        break;
        
    }

    /**********************/
    /* Checking channel B */
    /**********************/
    
    switch(channelStateB)
    {   
        case ST_START_B:    
        /* Searching for initial state */
            if(nextPinStateB == ST_PIN_00_B)
            {
                channelStateB = ST_WAIT_B;
                curPinStateB = nextPinStateB;
            }
        break;
            
        case ST_WAIT_B:
            if(nextPinStateB != curPinStateB)
            {
                diffB = 0;
                fixedDiffB = 0;
                
                switch(nextPinStateB)
                {
                    case ST_PIN_01_B:
                        channelStateB = ST_MEASURE_MINUS_B;
                        break;
                            
                    case ST_PIN_10_B:
                        channelStateB = ST_MEASURE_PLUS_B;
                        break;
                            
                    case ST_PIN_11_B:
                    default:
                        channelStateB = ST_START_B;
                        break;

                }                        

                curPinStateB = nextPinStateB;
            }
            break;
            
        case ST_MEASURE_PLUS_B:
        case ST_MEASURE_MINUS_B:
            if(nextPinStateB != curPinStateB)
            {   /* Switching to next state */
                channelStateB = ST_START_B;

                if(nextPinStateB == ST_PIN_11_B)
                {
                    if(diffB < (-1* maxDiffValue))
                    {
                        diffB = (-1* maxDiffValue);
                    }
            
                    if(diffB > maxDiffValue)
                    {
                        diffB = maxDiffValue;
                    }

                    fixedDiffB = diffB;
                }
                else
                {
                    diffB = 0;
                    fixedDiffB = 0;
                }                        

                curPinStateB = nextPinStateB;
            }
            break;
            
        default:
            break;
        
    }
}
