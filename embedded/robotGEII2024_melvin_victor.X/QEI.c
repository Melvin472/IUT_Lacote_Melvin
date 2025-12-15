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

// --- CONFIGURATION DES COEFFICIENTS ET SENS ---
// 0.00001620 correspond à votre conversion ticks -> mètres
// ASTUCE : Si le robot recule quand on lui demande d'avancer, mettez un '-' ici.
// ASTUCE : Si le robot tourne sur place, inversez le signe DE L'UN DES DEUX seulement.

#define COEFF_DROIT   ( 0.00001620)  // Testez POSITIF d'abord
#define COEFF_GAUCHE  ( 0.00001620)  // J'ai enlevé le '-' pour tester (SWPAB gère déjà une inversion)

// --- CONSTANTES TEMPORELLES ---
// Doit correspondre à la fréquence d'appel de QEIUpdateData (ex: 100Hz ou 250Hz)
#ifndef FREQ_ECH_QEI
#define FREQ_ECH_QEI 100.0 // Mettez ici la fréquence réelle de votre boucle while(1)
#endif
#define QEI_DELTA_T (1.0 / FREQ_ECH_QEI) 

static float QeiDroitPosition_T_1;
static float QeiDroitPosition;
static float QeiGauchePosition_T_1;
static float QeiGauchePosition;
static float delta_d;
static float delta_g;

void InitQEI1() {
    // SWPAB = 1 inverse le sens de comptage matériellement (A <-> B)
    QEI1IOCbits.SWPAB = 1; 
    QEI1GECL = 0xFFFF;
    QEI1GECH = 0xFFFF;
    QEI1CONbits.QEIEN = 1; // Enable QEI Module
}

void InitQEI2() {
    // SWPAB = 1 inverse le sens de comptage matériellement (A <-> B)
    QEI2IOCbits.SWPAB = 1; 
    QEI2GECL = 0xFFFF;
    QEI2GECH = 0xFFFF;
    QEI2CONbits.QEIEN = 1; // Enable QEI Module
}

void QEIUpdateData() {
    // 1. Sauvegarde des anciennes positions (t-1)
    QeiDroitPosition_T_1 = QeiDroitPosition;
    QeiGauchePosition_T_1 = QeiGauchePosition;

    // 2. Lecture des registres bruts (32 bits)
    long QEI1RawValue = POS1CNTL;
    QEI1RawValue += ((long) POS1HLD << 16);
    
    long QEI2RawValue = POS2CNTL;
    QEI2RawValue += ((long) POS2HLD << 16);
    
    // 3. Conversion en mètres avec gestion du signe
    QeiDroitPosition = COEFF_DROIT * (float)QEI1RawValue;
    QeiGauchePosition = COEFF_GAUCHE * (float)QEI2RawValue; 

    // 4. Calcul des deltas (Distance parcourue en un cycle)
    delta_d = QeiDroitPosition - QeiDroitPosition_T_1;
    delta_g = QeiGauchePosition - QeiGauchePosition_T_1;

    // 5. Calcul des Vitesses
    // ATTENTION : Pour la première itération, éviter les sauts géants
    static int firstRun = 1;
    if (firstRun) {
        delta_d = 0;
        delta_g = 0;
        firstRun = 0;
    }

    robotState.vitesseDroitFromOdometry = delta_d / QEI_DELTA_T;
    robotState.vitesseGaucheFromOdometry = delta_g / QEI_DELTA_T;
    
    // 6. Cinématique (Modèle Différentiel)
    robotState.vitesseLineaireFromOdometry = (robotState.vitesseDroitFromOdometry + robotState.vitesseGaucheFromOdometry) / 2.0;
    robotState.vitesseAngulaireFromOdometry = (robotState.vitesseDroitFromOdometry - robotState.vitesseGaucheFromOdometry) / DISTROUES;
        
    // 7. Odometrie (Intégration de la position X, Y, Theta)
    double angleMoyen = robotState.angleRadianFromOdometry + (robotState.vitesseAngulaireFromOdometry * QEI_DELTA_T / 2.0);
    
    robotState.xPosFromOdometry += (robotState.vitesseLineaireFromOdometry * cos(angleMoyen)) * QEI_DELTA_T;
    robotState.yPosFromOdometry += (robotState.vitesseLineaireFromOdometry * sin(angleMoyen)) * QEI_DELTA_T;
    robotState.angleRadianFromOdometry += robotState.vitesseAngulaireFromOdometry * QEI_DELTA_T;
    
    // 8. Normalisation de l'angle (-PI à +PI)
    while (robotState.angleRadianFromOdometry > PI)
        robotState.angleRadianFromOdometry -= 2.0 * PI;
    while (robotState.angleRadianFromOdometry < -PI)
        robotState.angleRadianFromOdometry += 2.0 * PI;
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