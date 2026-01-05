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

//Initialisation d?un timer 16 bits
unsigned long timestamp;
unsigned long tstop = 0;

void InitTimer1(void) {
    T1CONbits.TON = 0; // Disable Timer
    T1CONbits.TCS = 0; //clock source = internal clock
    IFS0bits.T1IF = 0; // Clear Timer Interrupt Flag
    IEC0bits.T1IE = 1; // Enable Timer interrupt
    T1CONbits.TON = 1; // Enable Timer
    SetFreqTimer1(250); // 250 Hz (4 ms) : FRÉQUENCE DU PID
}

// ** INTERRUPT HAUTE FRÉQUENCE (4ms) : TÂCHES CRITIQUES SEULEMENT **
void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void) {
    
    // --- 1. ACQUISITION & ODOMÉTRIE (Rapide) ---
    // Note: On suppose que updateSensorValues() gère la lecture et relance l'ADC
    // Nous retirons ADC1StartConversionSequence() d'ici pour éviter la redondance
    QEIUpdateData();
    updateSensorValues(); 
    Cap();
    
    // --- 2. LOGIQUE DE CONTRÔLE (Essentielle) ---
    UpdateTrajectory(); 
    UpdateAsservissement();
    PWMUpdateSpeed();
    
    // --- 3. DIVERS
    robotState.timeFrom++;
    
    // ATTENTION: TOUS LES APPELS UART ONT ÉTÉ DÉPLACÉS VERS _T4Interrupt
    
    // Nettoyage du flag
    IFS0bits.T1IF = 0;
}

// ... (InitTimer23 & _T3Interrupt inchangés) ...
void InitTimer23(void) {
    T3CONbits.TON = 0; // Stop any 16-bit Timer3 operation
    T2CONbits.TON = 0; // Stop any 16/32-bit Timer3 operation
    T2CONbits.T32 = 1; // Enable 32-bit Timer mode
    T2CONbits.TCS = 0; // Select internal instruction cycle clock
    T2CONbits.TCKPS = 0b00; // Select 1:1 Prescaler
    TMR3 = 0x00; // Clear 32-bit Timer (msw)
    TMR2 = 0x00; // Clear 32-bit Timer (lsw)
    PR3 = 0x0393; // Load 32-bit period value (msw) // a modif
    PR2 = 0x8700; // Load 32-bit period value (lsw) // a modif
    IPC2bits.T3IP = 0x01; // Set Timer3 Interrupt Priority Level
    IFS0bits.T3IF = 0; // Clear Timer3 Interrupt Flag
    IEC0bits.T3IE = 1; // Enable Timer3 interrupt
    T2CONbits.TON = 1; // Start 32-bit Timer
}

void __attribute__((interrupt, no_auto_psv)) _T3Interrupt(void) {
    IFS0bits.T3IF = 0;
}

void SetFreqTimer1(float freq) {
    T1CONbits.TCKPS = 0b01; //00 = 1:1 prescaler value
    if (FCY / freq > 65535) {
        T1CONbits.TCKPS = 0b01; //01 = 1:8 prescaler value
        if (FCY / freq / 8 > 65535) {
            T1CONbits.TCKPS = 0b10; //10 = 1:64 prescaler value
            if (FCY / freq / 64 > 65535) {
                T1CONbits.TCKPS = 0b11; //11 = 1:256 prescaler value
                PR1 = (int) (FCY / freq / 256);
            } else
                PR1 = (int) (FCY / freq / 64);
        } else
            PR1 = (int) (FCY / freq / 8);
    } else
        PR1 = (int) (FCY / freq);
}

void InitTimer4(void) {
    T4CONbits.TON = 0; // Disable Timer
    T4CONbits.TCS = 0; //clock source = internal clock
    IFS1bits.T4IF = 0; // Clear Timer Interrupt Flag
    IEC1bits.T4IE = 1; // Enable Timer interrupt
    T4CONbits.TON = 1; // Enable Timer
    SetFreqTimer4(100); // 100 Hz (10 ms) : FRÉQUENCE TÉLÉMÉTRIE
}

// ... (SetFreqTimer4 inchangé) ...

void SetFreqTimer4(float freq) {
    T4CONbits.TCKPS = 0b00; //00 = 1:1 prescaler value
    if (FCY / freq > 65535) {
        T4CONbits.TCKPS = 0b01; //01 = 1:8 prescaler value
        if (FCY / freq / 8 > 65535) {
            T4CONbits.TCKPS = 0b10; //10 = 1:64 prescaler value
            if (FCY / freq / 64 > 65535) {
                T4CONbits.TCKPS = 0b11; //11 = 1:256 prescaler value
                PR4 = (int) (FCY / freq / 256);
            } else
                PR4 = (int) (FCY / freq / 64);
        } else
            PR4 = (int) (FCY / freq / 8);
    } else
        PR4 = (int) (FCY / freq);
}

// ** INTERRUPT BASSE FRÉQUENCE (10ms) : TÂCHES UART/TÉLÉMÉTRIE **
void __attribute__((interrupt, no_auto_psv)) _T4Interrupt(void) {
    static unsigned int send_counter = 0;

    IFS1bits.T4IF = 0;
    
    // --- 1. GESTION DU TEMPS (si T4 est à 1ms, sinon ignorer) ---
    // Vous incrémentez ici, donc T4 semble être votre base de temps (1ms ou 10ms)
    // Si T4 est à 100 Hz (10ms), le timestamp s'incrémentera de 10.
    timestamp = timestamp + 1; // À vérifier si T4 est vraiment à 1ms ou 10ms
    tstop = tstop + 1;
    
    // --- 2. ENVOI DES DONNÉES (Lent) ---
    sendPidDonnees(); // Envoi des données PID à 100 Hz (10ms)

    // Envoi de la position (Télémétrie lourde) moins souvent
    if (send_counter++ >= 4) { // 100 Hz / 5 = 20 Hz (Toutes les 50ms)
        send_counter = 0;
        SendPositionData();
    }
}