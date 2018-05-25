/*
 * TestLogic.c
 *
 * Created: 24.05.2018 19:00:55
 * Author : Anctord
 */ 

#include <avr/io.h>

#define TIMER_PERIOD    7   // mks
#define MAX_ANGLE 90

/* Channel states types */
typedef enum {
    ST_START_A,
    ST_WAIT_A,
    ST_MEASURE_PLUS_A,
    ST_MEASURE_MINUS_A
} TChannelStateA;

TChannelStateA channelStateA;


volatile int powerPeriod;   // Sine Power Network time period in us
volatile int degreeTime;    // Time of 1' of network Sine (calculated)

volatile int fixedDiffA;

volatile int maxDiffTime;   // us
volatile int maxDiffValue;  

int main(void)
{
    unsigned char outValueA;

    fixedDiffA = 690;
    /* Replace with your application code */
    while (1) 
    {
        powerPeriod = 20000;
        
        degreeTime = powerPeriod / 360;
        maxDiffTime = MAX_ANGLE * degreeTime;
        maxDiffValue = maxDiffTime / TIMER_PERIOD;

        outValueA = ( ((long)(fixedDiffA + maxDiffValue)) << 8) / (maxDiffValue << 1);
        
        
        
        if(channelStateA == ST_START_A || channelStateA == ST_WAIT_A)
        {
            
            OCR1A = (unsigned char)outValueA ;    // Channel A: set difference = 0
        }

        

    }
}

