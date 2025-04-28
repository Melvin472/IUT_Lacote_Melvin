#include <math.h>

#define PI 3.14159265358979323846
#define DISTROUES 0.2812
#define FREQ_ECH_QEI 250  // Fréquence d'échantillonnage en Hz
#define POSITION_DATA 0x0061

// Structure pour stocker l'état du robot
typedef struct {
    double vitesseDroitFromOdometry;
    double vitesseGaucheFromOdometry;
    double vitesseLineaireFromOdometry;
    double vitesseAngulaireFromOdometry;
    double xPosFromOdometry;
    double yPosFromOdometry;
    double angleRadianFromOdometry;
    double xPosFromOdometry_1;
    double yPosFromOdometry_1;
    double angleRadianFromOdometry_1;
} RobotState;

// Variables globales pour les positions des roues
double QeiDroitPosition;
double QeiGauchePosition;
double QeiDroitPosition_T_1;
double QeiGauchePosition_T_1;
double delta_d;
double delta_g;

// Structure globale pour l'état du robot
RobotState robotState;

void QEIUpdateData() {
    // On sauvegarde les anciennes valeurs
    QeiDroitPosition_T_1 = QeiDroitPosition;
    QeiGauchePosition_T_1 = QeiGauchePosition;

    // On actualise les valeurs des positions
    long QEI1RawValue = POS1CNTL;
    QEI1RawValue += ((long)POS1HLD << 16);
    long QEI2RawValue = POS2CNTL;
    QEI2RawValue += ((long)POS2HLD << 16);

    // Conversion en mm (règle pour la taille des roues codeuses)
    QeiDroitPosition = 0.00001620 * QEI1RawValue;
    QeiGauchePosition = -0.00001620 * QEI2RawValue;

    // Calcul des deltas de position
    delta_d = QeiDroitPosition - QeiDroitPosition_T_1;
    delta_g = QeiGauchePosition - QeiGauchePosition_T_1;

    // Calcul des vitesses
    robotState.vitesseDroitFromOdometry = delta_d * FREQ_ECH_QEI;
    robotState.vitesseGaucheFromOdometry = delta_g * FREQ_ECH_QEI;
    robotState.vitesseLineaireFromOdometry = (robotState.vitesseDroitFromOdometry + robotState.vitesseGaucheFromOdometry) / 2.0;
    robotState.vitesseAngulaireFromOdometry = (robotState.vitesseDroitFromOdometry - robotState.vitesseGaucheFromOdometry) / DISTROUES;

    // Mise à jour du positionnement terrain à t-1
    robotState.xPosFromOdometry_1 = robotState.xPosFromOdometry;
    robotState.yPosFromOdometry_1 = robotState.yPosFromOdometry;
    robotState.angleRadianFromOdometry_1 = robotState.angleRadianFromOdometry;

    // Calcul des positions dans le référentiel du terrain
    robotState.xPosFromOdometry = robotState.xPosFromOdometry_1 + robotState.vitesseLineaireFromOdometry * cos(robotState.angleRadianFromOdometry_1) / FREQ_ECH_QEI;
    robotState.yPosFromOdometry = robotState.yPosFromOdometry_1 + robotState.vitesseLineaireFromOdometry * sin(robotState.angleRadianFromOdometry_1) / FREQ_ECH_QEI;
    robotState.angleRadianFromOdometry = robotState.angleRadianFromOdometry_1 + robotState.vitesseAngulaireFromOdometry / FREQ_ECH_QEI;

    // Ajustement de l'angle
    if (robotState.angleRadianFromOdometry > PI){robotState.angleRadianFromOdometry -= 2 * PI;}
    if (robotState.angleRadianFromOdometry < -PI){robotState.angleRadianFromOdometry += 2 * PI;}
}


void SendPositionData()
{
    unsigned char positionPayload[24];
    getBytesFromInt32(positionPayload, 0, timestamp);
    getBytesFromFloat(positionPayload, 4, (float)(robotState.xPosFromOdometry));
    getBytesFromFloat(positionPayload, 8, (float)(robotState.yPosFromOdometry));
    getBytesFromFloat(positionPayload, 12, (float)(robotState.angleRadianFromOdometry));
    getBytesFromFloat(positionPayload, 16, (float)(robotState.vitesseLineaireFromOdometry));
    getBytesFromFloat(positionPayload, 20, (float)(robotState.vitesseAngulaireFromOdometry));
    UartEncodeAndSendMessage(POSITION_DATA, 24, positionPayload);
}