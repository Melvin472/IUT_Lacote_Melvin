#ifndef ROBOT_H
#define ROBOT_H

#include "asservissement.h"  // Make sure this defines PidCorrector

// Define the robot state structure
typedef struct robotStateBITS {

    // Current task
    unsigned char taskEnCours;

    // Wheel speed commands
    float vitesseGaucheConsigne;
    float vitesseGaucheCommandeCourante;
    float vitesseDroiteConsigne;
    float vitesseDroiteCommandeCourante;

    // Telemeters / distance sensors
    float distanceTelemetreCentre;
    float distanceTelemetreGauche;
    float distanceTelemetreDroit;
    float distanceTelemetreExDroite;
    float distanceTelemetreExGauche;

    // Odometry
    float vitesseDroitFromOdometry;
    float vitesseGaucheFromOdometry;
    float vitesseLineaireFromOdometry;
    float vitesseAngulaireFromOdometry;
    float xPosFromOdometry_1;
    float yPosFromOdometry_1;
    float xPosFromOdometry;
    float yPosFromOdometry;
    float angleRadianFromOdometry_1;
    float angleRadianFromOdometry;
    float timeFrom;

    // Payloads / consignes
    char correcteursThetaXPayload[88];
    char consignes[8];

    // Speed correction
    float xCorrectionVitesse;
    float thetaCorrectionVitesse;
    float consigneVitesseLineaire;
    float consigneVitesseAngulaire;

    // --- Position ArUco ---
    float arucoX;
    float arucoY;
    float arucoDistance;

    // PID controllers
    PidCorrector PidX;
    PidCorrector PidTheta;

} ROBOT_STATE_BITS;

// Declare the global robot state as volatile
extern volatile ROBOT_STATE_BITS robotState;

#endif /* ROBOT_H */
