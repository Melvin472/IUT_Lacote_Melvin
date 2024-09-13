#include "ChipConfig.h"


#pragma config GWRP = OFF               
#pragma config GSS = OFF               
#pragma config GSSK = OFF               


#ifdef USE_CRYSTAL_OSCILLATOR
#pragma config FNOSC = PRIPLL           
#else
#pragma config FNOSC = FRCPLL           
#endif
#pragma config IESO = ON               


#ifdef USE_CRYSTAL_OSCILLATOR
#pragma config POSCMD = HS      
#pragma config OSCIOFNC = OFF           
#else
#pragma config POSCMD =  NONE          
#pragma config OSCIOFNC = ON           
#endif

#pragma config IOL1WAY = OFF             
#pragma config FCKSM = CSECMD           

#pragma config WDTPOST = PS32768        
#pragma config WDTPRE = PR128           
#pragma config PLLKEN = ON              
#pragma config WINDIS = OFF             
#pragma config FWDTEN = OFF           


#pragma config FPWRT = PWR128           
#pragma config BOREN = ON               
#pragma config ALTI2C1 = ON             
#pragma config ALTI2C2 = ON            


#pragma config ICS = PGD2               
#pragma config RSTPRI = PF              
#pragma config JTAGEN = OFF             


#pragma config AWRP = OFF               
#pragma config APL = OFF                
#pragma config APLK = OFF               



#include <xc.h>

void InitOscillator() {

    
    OSCTUNbits.TUN = 23;
    
    //Configuration for 60MIPS (120MHz), en réalité 119.76 MHz cf. feuille calcul Matlab
    PLLFBDbits.PLLDIV = 88; // M=65
    CLKDIVbits.PLLPOST = 0; // 
    CLKDIVbits.PLLPRE = 1; // 
    // Initiate Clock Switch to FRC oscillator with PLL (NOSC=0b001)
    __builtin_write_OSCCONH(0x01);
    __builtin_write_OSCCONL(OSCCON | 0x01);
    // Wait for Clock switch to occur
    while (OSCCONbits.COSC != 0b001);

    ACLKCON3bits.FRCSEL = 1; //1 = FRC is the clock source for APLL
    //0 = Auxiliary Oscillator or Primary Oscillator is the clock source for APLL (determined by ASRCSEL bit)        

    //Calcul cf. feuille Matlab ou au dessus
    ACLKCON3bits.APLLPRE = 0b001; //001 = Divided by 2
    //Division par 2, donc FAREF= F_IN/2 = 4MHz -> Conforme a 3MHz<FAREF<5.5MHz
    ACLKDIV3bits.APLLDIV = 0b111; //111 = 24
    //Multiplication par 24, donc FAVCO=(F_IN/2)*24 = 4*24 = 96MHz  60MHz < FVCO < 120MHz
    ACLKCON3bits.APLLPOST = 0b110; //110 = Divided by 2
    //Division par 2, donc FAVCO=((F_IN/2)*24)/2 = 48MHz  

    ACLKCON3bits.SELACLK = 1; //1 = Auxiliary PLL or oscillator provides the source clock for auxiliary clock divider
    //0 = Primary PLL provides the source clock for auxiliary clock divider

    ACLKCON3bits.ENAPLL = 1;
    while (ACLKCON3bits.APLLCK != 1);
    
    // Wait for PLL to lock
    while (OSCCONbits.LOCK != 1);

}
