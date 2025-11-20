/*
 * File: main.c
 * Author: Table 9 & Gemini
 * Version: Intégration Ghost + PID Autonome
 */

#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
#include <libpic30.h>

// Drivers Hardware
#include "ChipConfig.h"
#include "IO.h"
#include "timer.h"
#include "PWM.h"
#include "ADC.h"
#include "UART.h"
#include "QEI.h"

// Communication
#include "CB_TX1.h"
#include "CB_RX1.h"
#include "UART_protocol.h"

// Intelligence du Robot
#include "robot.h"
#include "ToolBox.h"
#include "main.h"
#include "GhostManager.h"    // <--- NOUVEAU
#include "asservissement.h"  // <--- NOUVEAU

// Variables Globales
float boundaryTelemetre = 100;
extern unsigned long timestamp; // Compteur de temps (ms) géré par timer.c

// ============================================================================
// GESTION DES CAPTEURS (Gardé pour l'affichage LED)
// ============================================================================
void updateSensorValues() {
    if (ADCIsConversionFinished() == 1) {
        ADCClearConversionFinishedFlag();
        unsigned int *result = ADCGetResult();
        
        float volts;
        
        volts = ((float) result[0]) * 3.3 / 4096;
        robotState.distanceTelemetreExGauche = Min(34 / volts - 5, boundaryTelemetre);
        
        volts = ((float) result[1]) * 3.3 / 4096;
        robotState.distanceTelemetreGauche = Min(34 / volts - 5, boundaryTelemetre);
        
        volts = ((float) result[2]) * 3.3 / 4096;
        robotState.distanceTelemetreCentre = Min(34 / volts - 5, boundaryTelemetre);
        
        volts = ((float) result[3]) * 3.3 / 4096;
        robotState.distanceTelemetreDroit = Min(34 / volts - 5, boundaryTelemetre);
        
        volts = ((float) result[4]) * 3.3 / 4096;
        robotState.distanceTelemetreExDroite = Min(34 / volts - 5, boundaryTelemetre);
        
        // Relance ADC immédiatement après la lecture si l'on n'utilise pas le timer
        ADC1StartConversionSequence(); 
    }
}

void Cap() {
    // Gestion des LEDs en fonction de la distance (Feedback visuel)
    LED_VERTE_1   = (robotState.distanceTelemetreExDroite < 24);
    LED_ROUGE_1   = (robotState.distanceTelemetreDroit < 38);
    LED_ORANGE_1  = (robotState.distanceTelemetreCentre < 38);
    LED_BLEUE_1   = (robotState.distanceTelemetreGauche < 30);
    LED_BLANCHE_1 = (robotState.distanceTelemetreExGauche < 24);
}

// ============================================================================
// MAIN
// ============================================================================
int main(void) {

    // 1. Initialisation Hardware
    InitOscillator();
    InitIO();
    InitTimer1();
    InitTimer4();
    InitTimer23(); // Gestion du Timestamp
    InitPWM();
    InitADC1();
    InitUART();
    InitQEI1();    // Codeur Moteur 1
    InitQEI2();    // Codeur Moteur 2
    
    // 2. Initialisation Intelligence (Cerveau)
    // Charge les PID (Kp, Ki...) "en dur" pour ne pas attendre le PC
    InitAsservissement(); 
    
    // Prépare la trajectoire (Waypoints)
    InitTrajectoryGenerator(); 
    
    // Active l'ADC pour la première conversion
    // CORRECTION APPLIQUÉE ICI: Remplacement de ADCStartConversion()
    ADC1StartConversionSequence();

    unsigned long lastLoopTime = 0;

    // 3. Boucle Principale
    while(1) {
        
        // Gestion de la fréquence de boucle : 100 Hz (toutes les 10ms)
        // C'est important pour que le PID et la trajectoire soient stables
        if ((timestamp - lastLoopTime) >= 10) {
            lastLoopTime = timestamp;

            // --- A. Mise à jour Capteurs & Odométrie ---
            updateSensorValues(); // Lecture Télémètres (qui relance la conversion)
            Cap();                // Mise à jour LEDs
            
            // NOTE IMPORTANTE:
            // Pour des systèmes temps réel, les fonctions ci-dessous (B, C, D) 
            // devraient idéalement être déplacées dans l'interruption du Timer1/Timer4
            // qui tourne à 100Hz pour garantir la stabilité.
            QEIUpdateData();
            // --- B. Mise à jour du Ghost (Consigne) ---
            UpdateTrajectory();

            // --- C. Mise à jour de l'Asservissement (Commande) ---
            UpdateAsservissement();
            
            // --- D. Envoi Télémétrie (Pour Debug PC) ---
            static int frameCount = 0;
            if (frameCount++ >= 5) {
                SendGhostData();
                frameCount = 0;
            }
        }

        // --- E. Gestion des ordres PC (Asynchrone) ---
        if(CB_RX1_IsDataAvailable()) {
            unsigned char data = CB_RX1_Get();
            UartDecodeMessage(data);
        }
    }

    return 0;
}