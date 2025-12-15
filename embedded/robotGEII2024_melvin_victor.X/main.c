/*
 * File: main.c
 * Author: Table 9 & Gemini
 * Version: MASTER - Contrôle dans le Main (Boucle 100Hz)
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
#include "GhostManager.h"    
#include "asservissement.h"  

// Variables Globales
float boundaryTelemetre = 100;
extern unsigned long timestamp; // Compteur de temps (ms) géré par timer.c

// ============================================================================
// GESTION DES CAPTEURS
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
        
        // Relance ADC immédiatement
        ADC1StartConversionSequence(); 
    }
}

void Cap() {
    // Feedback visuel
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
    
    // 2. Initialisation Intelligence
    InitAsservissement();      // Charge les gains PID
    InitTrajectoryGenerator(); // Prépare le premier Waypoint
    
    ADC1StartConversionSequence();

    unsigned long lastLoopTime = 0;

    // 3. Boucle Principale
    while(1) {
        
        // Gestion de la fréquence de boucle : 100 Hz (toutes les 10ms)
        if ((timestamp - lastLoopTime) >= 10) {
            lastLoopTime = timestamp;

            // ====================================================
            // ETAPE 1 : PERCEPTION (Où suis-je ?)
            // ====================================================
            QEIUpdateData(); // Lit les encodeurs et met à jour X, Y, Theta
            updateSensorValues();
            Cap();
            
            // --- DEBUG ODOMETRIE ACTIVE ---
            // Utile pour vérifier si le robot "voit" qu'il tourne
            static int odoDebug = 0;
            if (odoDebug++ > 20) { // Tous les 200ms (20 * 10ms)
                unsigned char debugBuf[64];
                // Affiche X, Y et l'Angle en Degrés
                sprintf((char*)debugBuf, "X:%.2f Y:%.2f Ang:%.1f\r\n", 
                        (double)robotState.xPosFromOdometry, 
                        (double)robotState.yPosFromOdometry, 
                        (double)(robotState.angleRadianFromOdometry * 180.0 / 3.14159));
                // Il faut inclure <string.h> tout en haut du fichier si ce n'est pas fait
SendMessage((unsigned char*)debugBuf, strlen(debugBuf));
                odoDebug = 0;
            }

            // ====================================================
            // ETAPE 2 : DECISION (Où dois-je aller ?)
            // ====================================================
            UpdateTrajectory();

            // ====================================================
            // ETAPE 3 : ACTION (Comment j'y vais ?)
            // ====================================================
            UpdateAsservissement();
            
            // ====================================================
            // ETAPE 4 : TELEMETRIE (Pour l'Interface PC)
            // ====================================================
            static int frameCount = 0;
            if (frameCount++ >= 5) { // 20Hz (toutes les 50ms)
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