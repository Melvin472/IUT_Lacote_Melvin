#include QEI.h
#define DISTROUES 0.2812
#define L 0.3 // Distance entre les roues (à ajuster selon votre robot)
#define FREQ_ECH_QEI 100 // Fréquence d'échantillonnage en Hz (à ajuster selon votre configuration)

void QEIUpdateData() {
    // On sauvegarde les anciennes valeurs
    QeiDroitPosition_T_1 = QeiDroitPosition;
    QeiGauchePosition_T_1 = QeiGauchePosition;

    // On actualise les valeurs des positions
    long QEI1RawValue = POS1CNTL;
    QEI1RawValue += ((long)POS1HLD << 16);
    long QEI2RawValue = POS2CNTL;
    QEI2RawValue += ((long)POS2HLD << 16);

    // Conversion en mm (réglée pour la taille des roues codeuses)
    QeiDroitPosition = 0.00001620 * QEI1RawValue;
    QeiGauchePosition = -0.00001620 * QEI2RawValue;

    // Calcul des deltas de position
    delta_d = QeiDroitPosition - QeiDroitPosition_T_1;
    delta_g = QeiGauchePosition - QeiGauchePosition_T_1;

    // Calcul des vitesses
    robotState.vitesseDroitFromOdometry = delta_d * FREQ_ECH_QEI;
    robotState.vitesseGaucheFromOdometry = delta_g * FREQ_ECH_QEI;

    // Calcul des vitesses linéaire et angulaire
    robotState.vitesseLineaireFromOdometry = (robotState.vitesseDroitFromOdometry + robotState.vitesseGaucheFromOdometry) / 2.0;
    robotState.vitesseAngulaireFromOdometry = (robotState.vitesseDroitFromOdometry - robotState.vitesseGaucheFromOdometry) / L;

    // Mise à jour du positionnement terrain à t-1
    robotState.xPosFromOdometry_1 = robotState.xPosFromOdometry;
    robotState.yPosFromOdometry_1 = robotState.yPosFromOdometry;
    robotState.angleRadianFromOdometry_1 = robotState.angleRadianFromOdometry;

    // Calcul des positions dans le référentiel du terrain
    robotState.xPosFromOdometry += robotState.vitesseLineaireFromOdometry * cos(robotState.angleRadianFromOdometry) / FREQ_ECH_QEI;
    robotState.yPosFromOdometry += robotState.vitesseLineaireFromOdometry * sin(robotState.angleRadianFromOdometry) / FREQ_ECH_QEI;
    robotState.angleRadianFromOdometry += robotState.vitesseAngulaireFromOdometry / FREQ_ECH_QEI;

    // Normalisation de l'angle dans l'intervalle [-PI, PI]
    if (robotState.angleRadianFromOdometry > PI)
        robotState.angleRadianFromOdometry -= 2 * PI;
    if (robotState.angleRadianFromOdometry < -PI)
        robotState.angleRadianFromOdometry += 2 * PI;
}

void InitQEI1() {
    QEI1IOCbits.SWPAB = 1; // QEAx et QEBx sont échangés
    QEI1GECL = 0xFFFF;
    QEI1GECH = 0xFFFF;
    QEI1CONbits.QEIEN = 1; // Activer le module QEI
}

void InitQEI2() {
    QEI2IOCbits.SWPAB = 1; // QEAx et QEBx ne sont pas échangés
    QEI2GECL = 0xFFFF;
    QEI2GECH = 0xFFFF;
    QEI2CONbits.QEIEN = 1; // Activer le module QEI
}
