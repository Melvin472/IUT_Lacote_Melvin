/*
 * File:   GhostManager.c
 * Objectif : Avancer automatiquement vers un ArUco (T3)
 */

#include <math.h>
#include "GhostManager.h"
#include "Robot.h"
#include "utilities.h"
#include <xc.h>

/* ==================== CONSTANTES ==================== */

#ifndef PI
#define PI 3.14159265358979323846
#endif

#define DISTANCE_STOP       0.10   // 10 cm : arrêt devant l?ArUco
#define ANGLE_TOLERANCE     0.05   // ~3°
#define MAX_LINEAR_SPEED    0.15   // m/s
#define MAX_ANGULAR_SPEED   0.8    // rad/s

#define KP_LINEAR   0.8
#define KP_ANGULAR  2.0

/* ==================== VARIABLES EXTERNES ==================== */
/* Position du fantôme pour UART (x, y, theta) */

extern volatile ROBOT_STATE_BITS robotState;

/* Données ArUco (remplies par l?UART) */
extern float arucoX;
extern float arucoY;
extern float arucoDistance;

/* ==================== INITIALISATION ==================== */

void InitTrajectoryGenerator(void)
{
    robotState.consigneVitesseLineaire = 0.0;
    robotState.consigneVitesseAngulaire = 0.0;
}

/* ==================== MISE À JOUR ==================== */

void UpdateTrajectory(void)
{
    double distance = arucoDistance;
    double angleToTarget = atan2(arucoY, arucoX);

    /* ===== ARRET SI TROP PROCHE OU PAS DE CIBLE ===== */
    if (distance <= DISTANCE_STOP || distance <= 0.0) {
        robotState.consigneVitesseLineaire = 0.0;
        robotState.consigneVitesseAngulaire = 0.0;
        return;
    }

    /* ===== VITESSE ANGULAIRE ===== */
    double w = KP_ANGULAR * angleToTarget;

    if (w > MAX_ANGULAR_SPEED)  w = MAX_ANGULAR_SPEED;
    if (w < -MAX_ANGULAR_SPEED) w = -MAX_ANGULAR_SPEED;

    /* ===== VITESSE LINEAIRE ===== */
    double v = KP_LINEAR * distance;

    /* Si on est mal aligné, on ralentit */
    if (fabs(angleToTarget) > 0.3) {
        v *= 0.3;
    }

    if (v > MAX_LINEAR_SPEED) v = MAX_LINEAR_SPEED;
    if (v < 0.0) v = 0.0;

    /* ===== CONSIGNES ===== */
    robotState.consigneVitesseLineaire = v;
    robotState.consigneVitesseAngulaire = w;
    
}

/* ==================== FONCTIONS INUTILES DÉSORMAIS ==================== */
/* Gardées pour compatibilité */

TrajectoryState GetCurrentTrajectoryState(void)
{
    return ADVANCING;
}

int GetCurrentWaypointIndex(void)
{
    return 0;
}

void ResetTrajectory(void)
{
    InitTrajectoryGenerator();
}