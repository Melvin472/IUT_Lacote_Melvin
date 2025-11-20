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

// Prototype local si non défini dans Utilities.h
float LimitToIntervalBis(float value, float lowLimit, float highLimit);

/* ====================================================================
 * FONCTION MODIFIÉE : Initialisation des PID avec des valeurs stables
 * ==================================================================== */
void InitAsservissement(void) {
    
    // --- PID LINEAIRE (X) ---
    // Objectif : Atteindre la vitesse linéaire de consigne (ex: 0.2 m/s).
    // Kp plus élevé, Ki modéré. Limite de sortie moteur à 100%.
    // Les valeurs Kp=3.0, Ki=120.0 étaient beaucoup trop agressives et donnaient une vitesse excessive.
    // NOUVEAU RÉGLAGE STABLE (À affiner) :
    SetupPidAsservissement(&robotState.PidX, 
                           /* Kp */ 3.0, 
                           /* Ki */ 120.0, 
                           /* Kd */ 0.0, 
                           /* CorrP_Max (Limit Kp*Epsilon) */ 100.0, 
                           /* CorrI_Max (Limit Ki*Integral) */ 100.0, 
                           /* CorrD_Max (Limit Kd*Deriv) */ 100.0);

    // --- PID ANGULAIRE (Theta) ---
    // Objectif : Corriger la déviation angulaire.
    // Kp est la cause principale de l'oscillation (tourne sur lui-même).
    // Ki est la cause du 'windup' si on bloque. Kd est souvent non nécessaire.
    // Les valeurs Kp=3.0, Ki=120.0 étaient extrêmement instables pour l'angle.
    // NOUVEAU RÉGLAGE STABLE (À affiner) :
    SetupPidAsservissement(&robotState.PidTheta, 
                           /* Kp */ 3.0, 
                           /* Ki */ 120.0, 
                           /* Kd */ 0.0, 
                           /* CorrP_Max */ 100.0, 
                           /* CorrI_Max */ 100.0, 
                           /* CorrD_Max */ 100.0);
}

// Configuration générique d'un correcteur PID
void SetupPidAsservissement(volatile PidCorrector* PidCorr, double Kp, double Ki, double Kd, double proportionelleMax, double integralMax, double deriveeMax) {
    PidCorr->Kp = Kp;
    PidCorr->erreurProportionelleMax = proportionelleMax;
    PidCorr->Ki = Ki;
    PidCorr->erreurIntegraleMax = integralMax;
    PidCorr->Kd = Kd;
    PidCorr->erreurDeriveeMax = deriveeMax;
    
    // Initialisation des mémoires à 0
    PidCorr->erreurIntegrale = 0;
    PidCorr->epsilon_1 = 0;
    PidCorr->erreur = 0;
    PidCorr->corrP = 0;
    PidCorr->corrI = 0;
    PidCorr->corrD = 0;
}

// Calculateur PID
double Correcteur(volatile PidCorrector* PidCorr, double erreur) {
    PidCorr->erreur = erreur;
    
    // Partie Proportionnelle
    // Correction: utilisation de la borne directement sur l'erreur * Kp
    PidCorr->corrP = PidCorr->Kp * erreur;
    PidCorr->corrP = LimitToIntervalBis(PidCorr->corrP, -PidCorr->erreurProportionelleMax, PidCorr->erreurProportionelleMax);

    // Partie Intégrale
    PidCorr->erreurIntegrale += erreur / FREQ_ECH_QEI;
    
    // Anti-windup: On borne l'intégrale
    double maxI_bound = (PidCorr->Ki != 0) ? (PidCorr->erreurIntegraleMax / PidCorr->Ki) : 0;
    PidCorr->erreurIntegrale = LimitToIntervalBis(PidCorr->erreurIntegrale, -maxI_bound, maxI_bound);
    
    PidCorr->corrI = PidCorr->Ki * PidCorr->erreurIntegrale;

    // Partie Dérivée
    double erreurDerivee = (erreur - PidCorr->epsilon_1) * FREQ_ECH_QEI;
    
    // Correction: application de la borne sur la correction dérivée
    PidCorr->corrD = erreurDerivee * PidCorr->Kd;
    PidCorr->corrD = LimitToIntervalBis(PidCorr->corrD, -PidCorr->erreurDeriveeMax, PidCorr->erreurDeriveeMax);

    PidCorr->epsilon_1 = erreur;
    
    return PidCorr->corrP + PidCorr->corrI + PidCorr->corrD;
}

// Boucle principale d'asservissement (A appeler périodiquement)
void UpdateAsservissement() {
    // Calcul des erreurs
    robotState.PidX.erreur = robotState.consigneVitesseLineaire - robotState.vitesseLineaireFromOdometry;
    robotState.PidTheta.erreur = robotState.consigneVitesseAngulaire - robotState.vitesseAngulaireFromOdometry;
    
    // Calcul des corrections
    robotState.xCorrectionVitesse = Correcteur(&robotState.PidX, robotState.PidX.erreur);
    robotState.thetaCorrectionVitesse = Correcteur(&robotState.PidTheta, robotState.PidTheta.erreur);
    
    // Envoi aux moteurs
    PWMSetSpeedConsignePolaire(robotState.xCorrectionVitesse, robotState.thetaCorrectionVitesse);
}

// ... (sendPidDonnees() et LimitToIntervalBis() inchangés) ...

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