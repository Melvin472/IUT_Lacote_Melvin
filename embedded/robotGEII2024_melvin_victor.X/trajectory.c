#include "trajectory.h"
#include "robot.h"
#include "Utilities.h"
#include "UART_Protocol.h"
#include "math.h"
#include "timer.h"
#include "QEI.h"
#include <stdio.h>

extern unsigned long timestamp;


volatile GhostPosition ghostPosition;

double maxAngularSpeed = 2.0;      // [rad/s]
double angularAccel = 5.0;         // [rad/s²]
double maxLinearSpeed = 1.0;       // [m/s]
double linearAccel = 0.5;          // [m/s²]

int current_state = IDLE;



// Normalise un angle entre -PI et +PI
double ModuloByAngle(double from, double to)
{
    double diff = fmod(to - from + PI, 2 * PI);
    if (diff < 0)
        diff += 2 * PI;
    return diff - PI;
}


void InitTrajectoryGenerator(void)
{
    ghostPosition.x = 0.0;
    ghostPosition.y = 0.0;
    ghostPosition.theta = 0.0;
    ghostPosition.linearSpeed = 0.0;
    ghostPosition.angularSpeed = 0.0;
    ghostPosition.targetX = 0.0;
    ghostPosition.targetY = 0.0;
    ghostPosition.angleToTarget = 0.0;
    ghostPosition.distanceToTarget = 0.0;

    current_state = IDLE;
}


void UpdateTrajectory(void)
{
    // 1?? Calcul des grandeurs initiales
    double thetaWaypoint = atan2(ghostPosition.targetY - ghostPosition.y,
                                 ghostPosition.targetX - ghostPosition.x);
    double thetaRestant = ModuloByAngle(ghostPosition.theta, thetaWaypoint);
    ghostPosition.angleToTarget = thetaRestant;

    double thetaArret = (ghostPosition.angularSpeed * ghostPosition.angularSpeed) / (2.0 * angularAccel);
    double incrementTheta = ghostPosition.angularSpeed / FREQ_ECH_QEI;

    // 2?? Gestion des états
    if (current_state == IDLE)
    {
        // Le ghost est à l'arrêt
        ghostPosition.angularSpeed = 0.0;
        ghostPosition.linearSpeed = 0.0;
        robotState.consigneVitesseAngulaire = 0.0;
        robotState.consigneVitesseLineaire = 0.0;
    }

    else if (current_state == ROTATING)
    {
        if (ghostPosition.angularSpeed < 0)
            thetaArret = -thetaArret;

        if ((((thetaArret >= 0) && (thetaRestant >= 0)) ||
             ((thetaArret <= 0) && (thetaRestant <= 0))) &&
            (fabs(thetaRestant) >= fabs(thetaArret)))
        {
            // On accélère
            if (thetaRestant > 0)
                ghostPosition.angularSpeed = Min(ghostPosition.angularSpeed + angularAccel / FREQ_ECH_QEI, maxAngularSpeed);
            else
                ghostPosition.angularSpeed = Max(ghostPosition.angularSpeed - angularAccel / FREQ_ECH_QEI, -maxAngularSpeed);
        }
        else
        {
            // On freine
            if (ghostPosition.angularSpeed > 0)
                ghostPosition.angularSpeed = Max(ghostPosition.angularSpeed - angularAccel / FREQ_ECH_QEI, 0);
            else if (ghostPosition.angularSpeed < 0)
                ghostPosition.angularSpeed = Min(ghostPosition.angularSpeed + angularAccel / FREQ_ECH_QEI, 0);

            if (fabs(thetaRestant) < fabs(incrementTheta))
                incrementTheta = thetaRestant;
        }

        ghostPosition.theta += incrementTheta;

        robotState.consigneVitesseAngulaire = ghostPosition.angularSpeed;
        robotState.consigneVitesseLineaire = 0.0;

        if (fabs(thetaRestant) < 0.01 && fabs(ghostPosition.angularSpeed) < 0.001)
        {
            ghostPosition.angularSpeed = 0.0;
            ghostPosition.theta = thetaWaypoint; // Correction des erreurs d'arrondi
            current_state = IDLE;
        }
    }

    SendGhostData();
}


void SendGhostData(void)
{
    unsigned char ghostPayload[32];
    getBytesFromInt32(ghostPayload, 0, timestamp);
    getBytesFromFloat(ghostPayload, 4, (float)ghostPosition.angleToTarget);
    getBytesFromFloat(ghostPayload, 8, (float)ghostPosition.distanceToTarget);
    getBytesFromFloat(ghostPayload, 12, (float)ghostPosition.theta);
    getBytesFromFloat(ghostPayload, 16, (float)ghostPosition.angularSpeed);
    getBytesFromFloat(ghostPayload, 20, (float)ghostPosition.x);
    getBytesFromFloat(ghostPayload, 24, (float)ghostPosition.y);
    getBytesFromFloat(ghostPayload, 28, (float)ghostPosition.linearSpeed);

    UartEncodeAndSendMessage(GHOST_DATA, 32, ghostPayload);
}
