#include <xc.h>
#include "UART.h"
#include "main.h"
#include "ChipConfig.h"
#define BAUDRATE 115200
#define BRGVAL ((FCY/BAUDRATE)/4)-1

void InitUART(void) {
    U1MODEbits.STSEL = 0; 
    U1MODEbits.PDSEL = 0; 
    U1MODEbits.ABAUD = 0; 
    U1MODEbits.BRGH = 1; 
    U1BRG = BRGVAL; 
    U1STAbits.UTXISEL0 = 0; 
    U1STAbits.UTXISEL1 = 0;
    IFS0bits.U1TXIF = 0; 
    IEC0bits.U1TXIE = 1; 
    U1MODEbits.UARTEN = 1; 
    U1STAbits.UTXEN = 1; 
}



void SendMessageDirect(unsigned char* message, int length) {
    unsigned char i = 0;
    for (i = 0; i < length; i++) {
        while (U1STAbits.UTXBF); 
        U1TXREG = *(message)++; 
    }
}
/*
void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt(void) {
    IFS0bits.U1RXIF = 0; 

    if (U1STAbits.FERR == 1) {
        U1STAbits.FERR = 0; 
    }

    if (U1STAbits.OERR == 1) {
        U1STAbits.OERR = 0;  
    }

    while (U1STAbits.URXDA == 1) {  
        U1TXREG = U1RXREG;  
    }
}

void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt(void) {
    IFS0bits.U1RXIF = 0;  

    
    if (U1STAbits.FERR == 1) {
        U1STAbits.FERR = 0;  
    }
    if (U1STAbits.OERR == 1) {
        U1STAbits.OERR = 0;  
    }

    
    while (U1STAbits.URXDA == 1) {
        unsigned char receivedData = U1RXREG;  
        CB_RX1_Add(receivedData);  
    }
}
*/