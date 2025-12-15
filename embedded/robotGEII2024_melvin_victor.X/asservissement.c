#include "asservissement.h"
#include "QEI.h"
#include "timer.h"
#include "IO.h"
#include "UART_Protocol.h"
#include "UART.h"
#include "Utilities.h"
#include "robot.h"
#include <xc.h>
#include "PWM.h"

// ====================================================================
// CONFIGURATION DE SECURITE (A MODIFIER ICI)
// ====================================================================

// Si le robot tourne sur lui-même (toupie) au lieu d'avancer :
// Mettez 1 à la place de 0 (ou inversement) pour inverser le moteur droit logiciellement.
#define INVERSE_MOTEUR_DROIT 1 

// Si le moteur Gauche tourne à l'envers :
#define INVERSE_MOTEUR_GAUCHE 0

// Fréquence d'asservissement (Doit correspondre à votre Timer !)
#ifndef FREQ_ECH_QEI
#define FREQ_ECH_QEI 100.0
#endif

// ====================================================================

// Prototype local
float LimitToIntervalBis(float value, float lowLimit, float highLimit);

/* ====================================================================
 * INITIALISATION : VALEURS DOUCES (SAFE MODE)
 * ==================================================================== */
void InitAsservissement(void) {
    
    // --- PID LINEAIRE (X) ---
    // On commence DOUX. Kp=0.8 est suffisant pour bouger sans osciller.
    // Ki=5.0 permet de corriger l'erreur statique lentement mais sûrement.
    SetupPidAsservissement(&robotState.PidX, 
                           /* Kp */ 0.8, 
                           /* Ki */ 5.0, 
                           /* Kd */ 0.0, 
                           /* Max P */ 100.0, 
                           /* Max I */ 50.0,  // Limité pour éviter les accélérations folles
                           /* Max D */ 0.0);

    // --- PID ANGULAIRE (Theta) ---
    // L'angle est sensible. Kp trop fort = oscillation immédiate.
    SetupPidAsservissement(&robotState.PidTheta, 
                           /* Kp */ 0.8, 
                           /* Ki */ 2.0, 
                           /* Kd */ 0.0, 
                           /* Max P */ 100.0, 
                           /* Max I */ 50.0, 
                           /* Max D */ 0.0);
}

// Configuration générique d'un correcteur PID
void SetupPidAsservissement(volatile PidCorrector* PidCorr, double Kp, double Ki, double Kd, double proportionelleMax, double integralMax, double deriveeMax) {
    PidCorr->Kp = Kp;
    PidCorr->erreurProportionelleMax = proportionelleMax;
    PidCorr->Ki = Ki;
    PidCorr->erreurIntegraleMax = integralMax;
    PidCorr->Kd = Kd;
    PidCorr->erreurDeriveeMax = deriveeMax;
    
    // Reset mémoires
    PidCorr->erreurIntegrale = 0;
    PidCorr->epsilon_1 = 0;
    PidCorr->erreur = 0;
    PidCorr->corrP = 0;
    PidCorr->corrI = 0;
    PidCorr->corrD = 0;
}

// Calculateur PID Standard
double Correcteur(volatile PidCorrector* PidCorr, double erreur) {
    PidCorr->erreur = erreur;
    
    // 1. Proportionnel
    PidCorr->corrP = PidCorr->Kp * erreur;
    PidCorr->corrP = LimitToIntervalBis(PidCorr->corrP, -PidCorr->erreurProportionelleMax, PidCorr->erreurProportionelleMax);

    // 2. Intégral
    PidCorr->erreurIntegrale += erreur / FREQ_ECH_QEI;
    
    // Anti-windup (Bornage de l'intégrale)
    double maxI_bound = (PidCorr->Ki != 0) ? (PidCorr->erreurIntegraleMax / PidCorr->Ki) : 0;
    PidCorr->erreurIntegrale = LimitToIntervalBis(PidCorr->erreurIntegrale, -maxI_bound, maxI_bound);
    
    PidCorr->corrI = PidCorr->Ki * PidCorr->erreurIntegrale;

    // 3. Dérivé
    double erreurDerivee = (erreur - PidCorr->epsilon_1) * FREQ_ECH_QEI;
    PidCorr->corrD = erreurDerivee * PidCorr->Kd;
    PidCorr->corrD = LimitToIntervalBis(PidCorr->corrD, -PidCorr->erreurDeriveeMax, PidCorr->erreurDeriveeMax);

    PidCorr->epsilon_1 = erreur;
    
    return PidCorr->corrP + PidCorr->corrI + PidCorr->corrD;
}

// Boucle principale d'asservissement
void UpdateAsservissement() {
    // 1. Calcul des erreurs (Consigne - Mesure)
    robotState.PidX.erreur = robotState.consigneVitesseLineaire - robotState.vitesseLineaireFromOdometry;
    robotState.PidTheta.erreur = robotState.consigneVitesseAngulaire - robotState.vitesseAngulaireFromOdometry;
    
    // 2. Calcul des corrections (Sortie du PID en % PWM ou en commande moteur)
    robotState.xCorrectionVitesse = Correcteur(&robotState.PidX, robotState.PidX.erreur);
    robotState.thetaCorrectionVitesse = Correcteur(&robotState.PidTheta, robotState.PidTheta.erreur);
    
    // 3. Mixage (Transforme Linéaire/Angulaire en Gauche/Droite)
    // V_Gauche = V_Lin - V_Ang
    // V_Droit  = V_Lin + V_Ang
    float commandeG = robotState.xCorrectionVitesse - robotState.thetaCorrectionVitesse;
    float commandeD = robotState.xCorrectionVitesse + robotState.thetaCorrectionVitesse;

    // 4. GESTION DE L'INVERSION (FIX POUR LE ROBOT QUI TOURNE EN ROND)
    if (INVERSE_MOTEUR_GAUCHE) commandeG = -commandeG;
    if (INVERSE_MOTEUR_DROIT)  commandeD = -commandeD;

    // 5. Envoi aux moteurs
    // On suppose que PWMSetMotor prend une valeur entre -100 et +100 (ou similaire)
    PWMSetMotor1(commandeG); // Moteur Gauche
    PWMSetMotor2(commandeD); // Moteur Droit
}

// Envoi des données pour debug (UART)
void sendPidDonnees() {
    getBytesFromFloat(robotState.correcteursThetaXPayload, 0, robotState.PidX.erreur);
    getBytesFromFloat(robotState.correcteursThetaXPayload, 4, robotState.xCorrectionVitesse);

    getBytesFromFloat(robotState.correcteursThetaXPayload, 8, robotState.PidX.Kp);
    getBytesFromFloat(robotState.correcteursThetaXPayload, 12, robotState.PidX.corrP);
    getBytesFromFloat(robotState.correcteursThetaXPayload, 16, robotState.PidX.erreurProportionelleMax);
    getBytesFromFloat(robotState.correcteursThetaXPayload, 20, robotState.PidX.Ki);
    getBytesFromFloat(robotState.correcteursThetaXPayload, 24, robotState.PidX.corrI);
    getBytesFromFloat(robotState.correcteursThetaXPayload, 28, robotState.PidX.erreurIntegraleMax);
    getBytesFromFloat(robotState.correcteursThetaXPayload, 32, robotState.PidX.Kd);
    getBytesFromFloat(robotState.correcteursThetaXPayload, 36, robotState.PidX.corrD);
    getBytesFromFloat(robotState.correcteursThetaXPayload, 40, robotState.PidX.erreurDeriveeMax);

    getBytesFromFloat(robotState.correcteursThetaXPayload, 44, robotState.PidTheta.erreur);
    getBytesFromFloat(robotState.correcteursThetaXPayload, 48, robotState.thetaCorrectionVitesse);

    getBytesFromFloat(robotState.correcteursThetaXPayload, 52, robotState.PidTheta.Kp);
    getBytesFromFloat(robotState.correcteursThetaXPayload, 56, robotState.PidTheta.corrP);
    getBytesFromFloat(robotState.correcteursThetaXPayload, 60, robotState.PidTheta.erreurProportionelleMax);
    getBytesFromFloat(robotState.correcteursThetaXPayload, 64, robotState.PidTheta.Ki);
    getBytesFromFloat(robotState.correcteursThetaXPayload, 68, robotState.PidTheta.corrI);
    getBytesFromFloat(robotState.correcteursThetaXPayload, 72, robotState.PidTheta.erreurIntegraleMax);
    getBytesFromFloat(robotState.correcteursThetaXPayload, 76, robotState.PidTheta.Kd);
    getBytesFromFloat(robotState.correcteursThetaXPayload, 80, robotState.PidTheta.corrD);
    getBytesFromFloat(robotState.correcteursThetaXPayload, 84, robotState.PidTheta.erreurDeriveeMax);

    UartEncodeAndSendMessage(PidThetaXConf, 88, robotState.correcteursThetaXPayload);
}

float LimitToIntervalBis(float value, float lowLimit, float highLimit) {
    if (value > highLimit)
        value = highLimit;
    else if (value < lowLimit)
        value = lowLimit;
    return value;
}