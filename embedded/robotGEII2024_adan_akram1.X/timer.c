#include <xc.h>
#include "timer.h"
#include "IO.h"
#include "PWM.h"
#include "ADC.h"
#include "main.h"
#include "QEI.h"
#include "robot.h"
#include "asservissement.h"
#include "UART_Protocol.h"

// Variables globales
unsigned long timestamp = 0;
unsigned long tstop = 0;
volatile unsigned long millisCounter = 0;   // compteur en ms

// ====================== TIMER 1 (1 ms) =========================
void InitTimer1(void) {
    T1CONbits.TON = 0;     // Disable Timer
    T1CONbits.TCS = 0;     // Clock source = internal
    IFS0bits.T1IF = 0;     // Clear flag
    IEC0bits.T1IE = 1;     // Enable interrupt
    SetFreqTimer1(1000);   // 1 kHz = 1 ms
    T1CONbits.TON = 1;     // Start Timer
}

void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void) {
    static unsigned int send_counter = 0;

    IFS0bits.T1IF = 0;   // reset flag

    millisCounter++;    
    ADC1StartConversionSequence();
    PWMUpdateSpeed();
    QEIUpdateData();
    robotState.timeFrom = (float)millisCounter;

    send_counter++;
    if (send_counter >= 25) { 
        send_counter = 0;

        UartEncodeAndSendMessage(PidXConf, 24, (unsigned char*)robotState.correcteursXPayload);
        UartEncodeAndSendMessage(PidThetaConf, 24, (unsigned char*)robotState.correcteursThetaPayload);

        SendPositionData();
    }
}

// ====================== TIMER 2+3 (32 bits) =========================
void InitTimer23(void) {
    T3CONbits.TON = 0;
    T2CONbits.TON = 0;
    T2CONbits.T32 = 1;    // mode 32 bits
    T2CONbits.TCS = 0;    // internal clock
    T2CONbits.TCKPS = 0b00; // prescaler 1:1

    TMR3 = 0;
    TMR2 = 0;

    PR3 = 0x0393; 
    PR2 = 0x8700;

    IPC2bits.T3IP = 0x01;
    IFS0bits.T3IF = 0;
    IEC0bits.T3IE = 1;
    T2CONbits.TON = 1;
}

void __attribute__((interrupt, no_auto_psv)) _T3Interrupt(void) {
    IFS0bits.T3IF = 0;
    PWMUpdateSpeed();
}

// ====================== TIMER 4 =========================
void InitTimer4(void) {
    T4CONbits.TON = 0;
    T4CONbits.TCS = 0;  // internal clock
    IFS1bits.T4IF = 0;
    IEC1bits.T4IE = 1;
    SetFreqTimer4(100); // 100 Hz
    T4CONbits.TON = 1;
}

void __attribute__((interrupt, no_auto_psv)) _T4Interrupt(void) {
    IFS1bits.T4IF = 0;
    timestamp++;
    tstop++;
}

// ====================== UTILS =========================
void SetFreqTimer1(float freq) {
    T1CONbits.TCKPS = 0b00;
    if (FCY / freq > 65535) {
        T1CONbits.TCKPS = 0b01;
        if (FCY / freq / 8 > 65535) {
            T1CONbits.TCKPS = 0b10;
            if (FCY / freq / 64 > 65535) {
                T1CONbits.TCKPS = 0b11;
                PR1 = (int)(FCY / freq / 256);
            } else PR1 = (int)(FCY / freq / 64);
        } else PR1 = (int)(FCY / freq / 8);
    } else PR1 = (int)(FCY / freq);
}

void SetFreqTimer4(float freq) {
    T4CONbits.TCKPS = 0b00;
    if (FCY / freq > 65535) {
        T4CONbits.TCKPS = 0b01;
        if (FCY / freq / 8 > 65535) {
            T4CONbits.TCKPS = 0b10;
            if (FCY / freq / 64 > 65535) {
                T4CONbits.TCKPS = 0b11;
                PR4 = (int)(FCY / freq / 256);
            } else PR4 = (int)(FCY / freq / 64);
        } else PR4 = (int)(FCY / freq / 8);
    } else PR4 = (int)(FCY / freq);
}
