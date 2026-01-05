#include "robot.h"
#include "asservissement.h"
#include <stdio.h>
// Ex : robot.c ou PWM.c
#include "PWM.h"  // si tu as déjà des fonctions PWM pour tes moteurs
volatile ROBOT_STATE_BITS robotState;

void ApplyMotorCommand(float vitesseGauche, float vitesseDroite) {
    // Ici, on fixe directement les consignes de vitesse
    robotState.vitesseGaucheConsigne = vitesseGauche;
    robotState.vitesseDroiteConsigne = vitesseDroite;

    // Met à jour les PWM pour suivre la rampe d'accélération
    PWMUpdateSpeed();
}


void robotInit(void) {
    robotState.taskEnCours = 0;
    robotState.vitesseGaucheConsigne = 0.0f;
    robotState.vitesseDroiteConsigne = 0.0f;
    robotState.xPosFromOdometry = 0.0f;
    robotState.yPosFromOdometry = 0.0f;
    robotState.angleRadianFromOdometry = 0.0f;

    robotState.arucoX = 0.0f;
    robotState.arucoY = 0.0f;
    robotState.arucoDistance = 0.0f;

    SetupPidAsservissement(&robotState.PidX, 1.0, 0.01, 0.1, 10, 5, 2);
    SetupPidAsservissement(&robotState.PidTheta, 2.0, 0.02, 0.2, 20, 10, 5);
}

void robotControlLoop(void) {

    // Vérifie si un ArUco est détecté
    // Ici on suppose que arucoDistance > 0 signifie détection valide
    if (robotState.arucoDistance > 0.0f) {

        // Calcul de l'erreur en X (distance à l'ArUco)
        double erreurX = (double)robotState.arucoX;  
        // Calcul de l'erreur d'angle pour viser l'ArUco
        double erreurTheta = (double)robotState.arucoY; // ou angle relatif si disponible

        // Calcul des corrections PID
        double correctionX = Correcteur(&robotState.PidX, erreurX);
        double correctionTheta = Correcteur(&robotState.PidTheta, erreurTheta);

        // Conversion en vitesse gauche/droite
        robotState.vitesseGaucheConsigne  = (float)(correctionX - correctionTheta);
        robotState.vitesseDroiteConsigne  = (float)(correctionX + correctionTheta);

        // Limitation des vitesses max/min pour éviter que le robot aille trop vite
        robotState.vitesseGaucheConsigne = LimitToIntervalBis(robotState.vitesseGaucheConsigne, -100.0f, 100.0f);
        robotState.vitesseDroiteConsigne = LimitToIntervalBis(robotState.vitesseDroiteConsigne, -100.0f, 100.0f);

    } else {
        // Aucun ArUco détecté ? robot immobile
        robotState.vitesseGaucheConsigne = 0.0f;
        robotState.vitesseDroiteConsigne = 0.0f;
    }

    // Appliquer les consignes aux moteurs
    ApplyMotorCommand(robotState.vitesseGaucheConsigne, robotState.vitesseDroiteConsigne);
}


