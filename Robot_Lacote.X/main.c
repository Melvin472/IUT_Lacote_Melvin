#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
#include "ChipConfig.h"
#include "IO.h"
#include "timer.h"
#include "PWM.h"
#include"Robot.h"
#include"ADC.h"

int main (void){



InitOscillator();
InitIO();
InitTimer23();
InitTimer1();
InitPWM();
PWMSetSpeedConsigne(20,1);
PWMSetSpeedConsigne(20,0);

//PWMSetSpeed(-20, 1);

LED_BLANCHE_1 = 1;
LED_BLEUE_1 = 1;
LED_ORANGE_1 = 1;
LED_ROUGE_1 = 1;
LED_VERTE_1 = 1;
LED_BLANCHE_2 = 1;
LED_BLEUE_2 = 1;
LED_ORANGE_2 = 1;
LED_ROUGE_2 = 1;
LED_VERTE_2 = 1;

while(1)
{
    if(ADCIsConversionFinished()){
        ADCConversionFinishedFlag();
        unsigned int * result = ADCGetResult();
    }
} 
}