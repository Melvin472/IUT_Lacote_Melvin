#include "QEI.h"
#include "timer.h"
#include "IO.h"
#include "UART_Protocol.h"
#include "UART.h"
#include "math.h"
#include "Utilities.h"
#include "robot.h"
#include <xc.h>
#include "asservissement.h"

// --- CONSTANTE CRITIQUE ---
// Doit être définie dans un fichier .h (ex: robot.h ou ChipConfig.h) à 250.0
// Pour l'exemple, nous allons la supposer définie, ainsi que DISTROUES et PI.
// #define FREQ_ECH_QEI 250.0
#define QEI_DELTA_T (1.0 / FREQ_ECH_QEI) // Période d'échantillonnage (0.004s)

static float QeiDroitPosition_T_1;
static float QeiDroitPosition;
static float QeiGauchePosition_T_1;
static float QeiGauchePosition;
static float delta_d;
static float delta_g;

void InitQEI1() {
    QEI1IOCbits.SWPAB = 1; // QEAx and QEBx are swapped (vérifier si cela correspond au moteur Droit)
    QEI1GECL = 0xFFFF;
    QEI1GECH = 0xFFFF;
    QEI1CONbits.QEIEN = 1; // Enable QEI Module
}

void InitQEI2() {
    QEI2IOCbits.SWPAB = 1; // QEAx and QEBx are swapped (vérifier si cela correspond au moteur Gauche)
    QEI2GECL = 0xFFFF;
    QEI2GECH = 0xFFFF;
    QEI2CONbits.QEIEN = 1; // Enable QEI Module
}

void QEIUpdateData() {
    // On sauvegarde les anciennes valeurs
    QeiDroitPosition_T_1 = QeiDroitPosition;
    QeiGauchePosition_T_1 = QeiGauchePosition;

    // On actualise les valeurs des positions
    long QEI1RawValue = POS1CNTL;
    QEI1RawValue += ((long) POS1HLD << 16);
    long QEI2RawValue = POS2CNTL;
    QEI2RawValue += ((long) POS2HLD << 16);
    
    // Conversion en mm (regle pour la taille des roues codeuses)
    // IMPORTANT : On travaille directement avec les positions.
    QeiDroitPosition = 0.00001620 * QEI1RawValue;
    QeiGauchePosition = -0.00001620 * QEI2RawValue; // Le signe inverse est critique

    // Calcul des deltas de position
    delta_d = QeiDroitPosition - QeiDroitPosition_T_1;
    delta_g = QeiGauchePosition - QeiGauchePosition_T_1;

    // --- CORRECTION : Utilisation de QEI_DELTA_T (1/FREQ) pour la régularité ---
    
    // Calcul des vitesses (V = Delta Position / Delta T)
    robotState.vitesseDroitFromOdometry = delta_d / QEI_DELTA_T;
    robotState.vitesseGaucheFromOdometry = delta_g / QEI_DELTA_T;
    
    robotState.vitesseLineaireFromOdometry = (robotState.vitesseDroitFromOdometry + robotState.vitesseGaucheFromOdometry) / 2.0;
    robotState.vitesseAngulaireFromOdometry = (robotState.vitesseDroitFromOdometry - robotState.vitesseGaucheFromOdometry) / DISTROUES;
        
    // Calcul des positions dans le referentiel du terrain (Position += Vitesse * Delta T)
    robotState.xPosFromOdometry += (robotState.vitesseLineaireFromOdometry * cos(robotState.angleRadianFromOdometry)) * QEI_DELTA_T;
    robotState.yPosFromOdometry += (robotState.vitesseLineaireFromOdometry * sin(robotState.angleRadianFromOdometry)) * QEI_DELTA_T;
    robotState.angleRadianFromOdometry += robotState.vitesseAngulaireFromOdometry * QEI_DELTA_T;
    
    // Normalisation de l'angle
    if (robotState.angleRadianFromOdometry > PI)
        robotState.angleRadianFromOdometry -= 2 * PI;
    if (robotState.angleRadianFromOdometry < -PI)
        robotState.angleRadianFromOdometry += 2 * PI;
}

#define POSITION_DATA 0x0061

void SendPositionData() {
    unsigned char positionPayload[36];
    getBytesFromInt32(positionPayload, 0, timestamp);
    getBytesFromFloat(positionPayload, 4, (float) (robotState.xPosFromOdometry));
    getBytesFromFloat(positionPayload, 8, (float) (robotState.yPosFromOdometry));
    getBytesFromFloat(positionPayload, 12, (float) (robotState.angleRadianFromOdometry));
    getBytesFromFloat(positionPayload, 16, (float) (robotState.vitesseLineaireFromOdometry));
    getBytesFromFloat(positionPayload, 20, (float) (robotState.vitesseAngulaireFromOdometry));
    getBytesFromFloat(positionPayload, 24, (float) (robotState.timeFrom));
    getBytesFromFloat(positionPayload, 28, (float) (robotState.vitesseDroitFromOdometry));
    getBytesFromFloat(positionPayload, 32, (float) (robotState.vitesseGaucheFromOdometry));

    UartEncodeAndSendMessage(POSITION_DATA, 36, positionPayload);
}