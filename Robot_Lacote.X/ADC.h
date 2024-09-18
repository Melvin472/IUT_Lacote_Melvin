/* 
 * File:   ADC.h
 * Author: GEII Robot
 *
 * Created on 18 septembre 2024, 17:12
 */

#ifndef ADC_H

void InitADC1(void);
void ADC1StartConversionSequence();
void ADCClearConversionFinishedFlag(void);
unsigned int * ADCGetResult(void);
unsigned char ADCIsConversionFinished(void);


#endif


