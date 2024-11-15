#include <xc.h>
#include "CB_RX1.h"

#define CBRX1_BUFFER_SIZE 128

int cbRx1Head = 0;
int cbRx1Tail = 0;
unsigned char cbRx1Buffer[CBRX1_BUFFER_SIZE];

void CB_RX1_Add(unsigned char value) {
    if (CB_RX1_GetRemainingSize() > 0) {
        cbRx1Buffer[cbRx1Head] = value;
        cbRx1Head = (cbRx1Head + 1) % CBRX1_BUFFER_SIZE;
    }
}

unsigned char CB_RX1_Get(void) {
    unsigned char value = cbRx1Buffer[cbRx1Tail];
    cbRx1Tail = (cbRx1Tail + 1) % CBRX1_BUFFER_SIZE;
    return value;
}

unsigned char CB_RX1_IsDataAvailable(void) {
    return (cbRx1Head != cbRx1Tail);
}

int CB_RX1_GetDataSize(void) {
    return (cbRx1Head - cbRx1Tail + CBRX1_BUFFER_SIZE) % CBRX1_BUFFER_SIZE;
}

int CB_RX1_GetRemainingSize(void) {
    return CBRX1_BUFFER_SIZE - CB_RX1_GetDataSize() - 1;
}

void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt(void) {
    IFS0bits.U1RXIF = 0;  // Clear RX interrupt flag
    if (U1STAbits.FERR == 1) {
        U1STAbits.FERR = 0;  // Clear framing error
    }
    if (U1STAbits.OERR == 1) {
        U1STAbits.OERR = 0;  // Clear overrun error
    }

    while (U1STAbits.URXDA == 1) {
        CB_RX1_Add(U1RXREG);  // Add received data to buffer
    }
}
