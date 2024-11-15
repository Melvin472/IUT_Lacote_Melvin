#include <xc.h>
#include "UART.h"
#include "main.h"
#include "ChipConfig.h"
#define BAUDRATE 115200
#define BRGVAL ((FCY/BAUDRATE)/4)-1

void InitUART(void) {
    U1MODEbits.STSEL = 0; // 1-stop bit
    U1MODEbits.PDSEL = 0; // No Parity, 8-data bits
    U1MODEbits.ABAUD = 0; // Auto-Baud Disabled
    U1MODEbits.BRGH = 1; // High Speed mode
    U1BRG = BRGVAL; // Baud rate setting
    U1STAbits.UTXISEL0 = 0; // Interrupt after one Tx character is transmitted
    U1STAbits.UTXISEL1 = 0;
    IFS0bits.U1TXIF = 0; // clear TX interrupt flag
    IEC0bits.U1TXIE = 1; // Enable UART Tx interrupt
    U1MODEbits.UARTEN = 1; // Enable UART
    U1STAbits.UTXEN = 1; // Enable UART Tx
}



void SendMessageDirect(unsigned char* message, int length) {
    unsigned char i = 0;
    for (i = 0; i < length; i++) {
        while (U1STAbits.UTXBF); 
        U1TXREG = *(message)++; 
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
        U1TXREG = U1RXREG;  
    }
}

void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt(void) {
    IFS0bits.U1RXIF = 0;  // Efface le flag d'interruption RX

    // V�rification des erreurs de r�ception (parit�, erreur de d�passement)
    if (U1STAbits.FERR == 1) {
        U1STAbits.FERR = 0;  // Efface l'erreur de parit�
    }
    if (U1STAbits.OERR == 1) {
        U1STAbits.OERR = 0;  // Efface l'erreur de d�passement
    }

    // Tant que des donn�es sont disponibles, on les ajoute au buffer
    while (U1STAbits.URXDA == 1) {
        unsigned char receivedData = U1RXREG;  // Lire la donn�e du registre UART
        CB_RX1_Add(receivedData);  
    }
}
