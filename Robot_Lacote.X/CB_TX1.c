#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include "CB_TX1.h"

int cbTx1Head = 0;
int cbTx1Tail = 0;
unsigned char cbTx1Buffer[CBTX1_BUFFER_SIZE];
unsigned char isTransmitting = 0;

void SendMessage(unsigned char* message, int length)
{
    unsigned char i = 0;
    
    if (CB_TX1_GetRemainingSize() >= length)
    {
        for (i = 0; i < length; i++)
        {
            CB_TX1_Add(message[i]);
        }

        if (!CB_TX1_IsTranmitting())
        {
            SendOne();
        }
    }
}

void CB_TX1_Add(unsigned char value)
{
    int nextHead = (cbTx1Head + 1) % CBTX1_BUFFER_SIZE;
    
    
    if (nextHead != cbTx1Tail)
    {
        cbTx1Buffer[cbTx1Head] = value;
        cbTx1Head = nextHead;
    }
}

unsigned char CB_TX1_Get(void)
{
    unsigned char value = cbTx1Buffer[cbTx1Tail];
    cbTx1Tail = (cbTx1Tail + 1) % CBTX1_BUFFER_SIZE;
    return value;
}

unsigned char CB_TX1_IsTranmitting(void)
{
    return isTransmitting;
}

int CB_TX1_GetDataSize(void)
{
    int dataSize = (cbTx1Head - cbTx1Tail + CBTX1_BUFFER_SIZE) % CBTX1_BUFFER_SIZE;
    return dataSize;
}

int CB_TX1_GetRemainingSize(void)
{
    return CBTX1_BUFFER_SIZE - CB_TX1_GetDataSize() - 1;
}

void __attribute__((interrupt, no_auto_psv)) _U1TXInterrupt(void)
{
    IFS0bits.U1TXIF = 0; 
    
    if (cbTx1Tail != cbTx1Head)
    {
        SendOne();
    }
    else
    {
        isTransmitting = 0; 
    }
}

void SendOne(void)
{
    isTransmitting = 1;
    unsigned char value = CB_TX1_Get();
    U1TXREG = value; 
}

