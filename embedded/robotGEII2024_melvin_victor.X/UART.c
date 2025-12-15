#include <xc.h>
#include "UART.h"
#include "ChipConfig.h"
#define FCY 60000000
#define BAUDRATE 115200
#define BRGVAL ((FCY/BAUDRATE)/4)-1

// Déclaration externe nécessaire pour la fonction de décodage
extern void UartDecodeMessage(unsigned char c); 
// Assurez-vous que cette déclaration est dans UART.h

// ######################################################################
// Initialisation de l'UART 1 (Inchangée)
// ######################################################################
void InitUART(void) {
    U1MODEbits.STSEL = 0; // 1-stop bit
    U1MODEbits.PDSEL = 0; // No Parity, 8-data bits
    U1MODEbits.ABAUD = 0; // Auto-Baud Disabled
    U1MODEbits.BRGH = 1; // Low Speed mode
    U1BRG = BRGVAL; // BAUD Rate Setting

    U1STAbits.UTXISEL0 = 0; // Interrupt after one Tx character is transmitted
    U1STAbits.UTXISEL1 = 0;
    IFS0bits.U1TXIF = 0; // clear TX interrupt flag    
    IEC0bits.U1TXIE = 1; // Enable UART Tx interrupt

    U1STAbits.URXISEL = 0; // Interrupt after one RX character is received;
    IFS0bits.U1RXIF = 0; // clear RX interrupt flag
    IEC0bits.U1RXIE = 1; //  UART Rx interrupt
    
    U1MODEbits.UARTEN = 1; // Enable UART
    IPC2bits.U1RXIP = 7;
    U1STAbits.UTXEN = 1; // Enable UART Tx
}

// ######################################################################
// Initialisation de l'UART 4 (NOUVEAU)
// ######################################################################
void InitUART2(void) {
    U2MODEbits.STSEL = 0; // 1-stop bit
    U2MODEbits.PDSEL = 0; // No Parity, 8-data bits
    U2MODEbits.ABAUD = 0; // Auto-Baud Disabled
    U2MODEbits.BRGH = 1; // Low Speed mode
    U2BRG = BRGVAL; // BAUD Rate Setting (même que U1 : 115200)

    // Configuration de l'interruption de Transmission (TX)
    U2STAbits.UTXISEL0 = 0; // Interrupt after one Tx character is transmitted
    U2STAbits.UTXISEL1 = 0;
    IFS1bits.U2TXIF = 0; // clear TX interrupt flag (Note : U4 utilise IFS1)
    IEC1bits.U2TXIE = 1; // Enable UART Tx interrupt

    // Configuration de l'interruption de Réception (RX)
    U2STAbits.URXISEL = 0; // Interrupt after one RX character is received;
    IFS1bits.U2RXIF = 0; // clear RX interrupt flag (Note : U4 utilise IFS1)
    IEC1bits.U2RXIE = 1; //  UART Rx interrupt
    
    // Activation de l'UART 4
    U2MODEbits.UARTEN = 1; // Enable UART
    // IPC16bits.U4RXIP = 7; // Ajustez l'IP si nécessaire, mais cela peut varier
    U2STAbits.UTXEN = 1; // Enable UART Tx
}

// ######################################################################
// Interruption de Réception pour l'UART 4 (Pour recevoir la JEVOIS-PRO)
// ######################################################################
void __attribute__((interrupt, no_auto_psv)) _U2RXInterrupt(void) {
    IFS1bits.U2RXIF = 0; // clear RX interrupt flag 
    
    // Gérer les erreurs (Frame Error et Overrun Error)
    if (U2STAbits.FERR == 1) {
        U2STAbits.FERR = 0;
    }
    if (U2STAbits.OERR == 1) {
        U2STAbits.OERR = 0;
    }
    
    // Lire tous les caractères disponibles et les passer au décodeur de trame
    while (U2STAbits.URXDA == 1) {
        unsigned char received_char = U2RXREG;
        // La fonction UartDecodeMessage gère la machine d'état (0xFE, Payload, Checksum)
        UartDecodeMessage(received_char);
    }
}

// ######################################################################
// Fonction d'envoi direct (à ajuster si vous voulez l'utiliser sur U4)
// ######################################################################
void SendMessageDirect(unsigned char* message, int length) {
    unsigned char i = 0;
    for (i = 0; i < length; i++) {
        // Envoi sur UART 1
        while (U1STAbits.UTXBF); // wait while Tx buffer full
        U1TXREG = *(message)++; // Transmit one character
    }
}

// Si vous utilisez l'UART 4 pour le débogage occasionnel vers la JeVois (Tx), 
// vous devriez créer une fonction SendMessageDirect_U4 similaire.