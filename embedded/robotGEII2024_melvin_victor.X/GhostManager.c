/*
 * File:   GhostManager.c
 * Gestion des waypoints et trajectoire du robot
 * Version : Logique du Carré Corrigée et Séparée (FINAL)
 */

#include <math.h>
#include "GhostManager.h"
#include "Robot.h"      // Contient la structure ROBOT_STATE_BITS (avec position/angle réels)
#include "utilities.h"  // Contient Min/Max/Abs et PI
#include "UART_Protocol.h"
#include <xc.h>

extern unsigned long timestamp;
extern volatile ROBOT_STATE_BITS robotState;

volatile GhostPosition ghostPosition; // Représente la CONSIGNE de trajectoire

/* ==================== PARAMÈTRES DE CONTRÔLE ==================== */
// Augmentation de la vitesse et de l'accélération pour un mouvement plus dynamique
double maxAngularSpeed = 0.5;       // rad/s (Vitesse max de rotation)
double angularAccel = 2.5;          // rad/s^2 (Accélération angulaire)
double maxLinearSpeed = 0.1;        // m/s (Vitesse max d'avance - Augmenté pour l'exemple)
double linearAccel = 0.1;           // m/s^2 (Accélération linéaire - Augmenté pour l'exemple)

/* ==================== GESTION DES WAYPOINTS ==================== */
#define MAX_WAYPOINTS 4

// Trajectoire : Carré de 20cm x 20cm
//Waypoint_t waypoints[MAX_WAYPOINTS] = {
//    {0.2,  0.0,  0},  // W1 : Coin 1
//    {0.2,  0.2,  0},  // W2 : Coin 2
//    {0.0,  0.2,  0},  // W3 : Coin 3
//    {0.0,  0.0,  0}   // W4 : Retour à l'origine (Bouclage)
//};
Waypoint_t waypoints[MAX_WAYPOINTS] = {
    {0.2,  0.0,  0},  // W1 : Coin 1
};

static int waypointIndex = 0;
static TrajectoryState currentState = IDLE;

/* ==================== INITIALISATION ==================== */
void InitTrajectoryGenerator(void) {
    // Initialiser les valeurs du GHOST (consigne) à la position initiale
    ghostPosition.x = 0.0;
    ghostPosition.y = 0.0;
    ghostPosition.theta = 0.0;
    ghostPosition.linearSpeed = 0.0;
    ghostPosition.angularSpeed = 0.0;
    ghostPosition.targetX = 0.0;
    ghostPosition.targetY = 0.0;
    ghostPosition.state = IDLE;
    
    // Assurer que les valeurs réelles sont initialisées à 0.0 (si non fait dans QEI.c)
    robotState.xPosFromOdometry = 0.0;
    robotState.yPosFromOdometry = 0.0;
    robotState.angleRadianFromOdometry = 0.0;

    waypointIndex = 0;
    currentState = IDLE;
}

/* ==================== FONCTIONS UTILITAIRES ==================== */

// Calcule l'angle le plus court (-PI à +PI)
static double computeAngleDifference(double theta, double thetaTarget) {
    double diff = thetaTarget - theta;
    while (diff > PI) diff -= 2 * PI;
    while (diff < -PI) diff += 2 * PI;
    return diff;
}

// Rampe de vitesse trapézoïdale
static double rampSpeed(double currentSpeed, double targetSpeed, double accel, double dt) {
    double maxChange = accel * dt;
    if (currentSpeed < targetSpeed) {
        return Min(currentSpeed + maxChange, targetSpeed);
    } else if (currentSpeed > targetSpeed) {
        return Max(currentSpeed - maxChange, targetSpeed);
    }
    return currentSpeed;
}

/* ==================== MISE À JOUR DE LA TRAJECTOIRE (CORRIGÉE) ==================== */

void UpdateTrajectory(void) {
    // 1. Gestion du Temps (Delta T)
    static double lastUpdateTime = 0;
    double currentTime = timestamp / 1000.0; // Conversion ms -> s

    double dt = 0.01;
    if (lastUpdateTime != 0) {
        dt = currentTime - lastUpdateTime;
    }
    lastUpdateTime = currentTime;

    if (dt > 0.1 || dt <= 0.0) dt = 0.01;

    // --- MISE À JOUR CRITIQUE 1: Récupérer la position réelle du robot ---
    // Le GHOST doit suivre le ROBOT, pas s'auto-intégrer !
    // On met à jour la position fantôme (l'état actuel du système de suivi)
    // avec la position réelle fournie par l'odométrie (QEI.c).
    // Cela permet au générateur de trajectoire de travailler avec les valeurs réelles.
    ghostPosition.x = robotState.xPosFromOdometry;
    ghostPosition.y = robotState.yPosFromOdometry;
    ghostPosition.theta = robotState.angleRadianFromOdometry;
    
    // 2. Calculs Géométriques vers la cible actuelle
    double dx = ghostPosition.targetX - ghostPosition.x;
    double dy = ghostPosition.targetY - ghostPosition.y;

    double thetaTarget = atan2(dy, dx);

    if (currentState == LASTROTATE) {
         thetaTarget = 0.0; // Rotation finale vers 0 radian (vers l'axe X)
    }

    // --- MISE À JOUR CRITIQUE 2: angleDiff utilise l'angle réel ---
    // On utilise l'angle réel du robot (mis à jour juste avant)
    double angleDiff = computeAngleDifference(ghostPosition.theta, thetaTarget);
    // ou si vous préférez: double angleDiff = computeAngleDifference(robotState.angleRadianFromOdometry, thetaTarget);

    ghostPosition.angleToTarget = angleDiff;
    ghostPosition.distanceToTarget = sqrt(dx * dx + dy * dy);

    // 3. Calcul des distances de freinage (Doit utiliser la vitesse de consigne, ce qui est correct)
    double angularStopDist = (ghostPosition.angularSpeed * ghostPosition.angularSpeed) / (2 * angularAccel);
    double linearStopDist = (ghostPosition.linearSpeed * ghostPosition.linearSpeed) / (2 * linearAccel);

    // ============ MACHINE À ÉTAT ============

    switch (currentState) {

        case IDLE:
            if (MAX_WAYPOINTS > 0) {
                
                if (waypointIndex >= MAX_WAYPOINTS) {
                    waypointIndex = 0;
                }
                
                Waypoint_t nextWay = waypoints[waypointIndex++];
                ghostPosition.targetX = nextWay.x;
                ghostPosition.targetY = nextWay.y;
                
                // Recalculer les différences avec la NOUVELLE cible
                dx = ghostPosition.targetX - ghostPosition.x;
                dy = ghostPosition.targetY - ghostPosition.y;
                ghostPosition.distanceToTarget = sqrt(dx * dx + dy * dy);
                
                // Si la distance est trop petite, on charge le WP suivant directement
                if (ghostPosition.distanceToTarget < DISTANCE_TOLERANCE) {
                    // On ne change pas d'état, on restera IDLE jusqu'à ce qu'un WP valide soit chargé
                    currentState = IDLE; 
                    // IMPORTANT : le waypointIndex a déjà été incrémenté, donc le prochain appel lira le suivant.
                }
                // Si ce n'est pas trop proche, on démarre la rotation
                else if (nextWay.isLastRotation) {
                    currentState = LASTROTATE;
                } else {
                    currentState = ROTATING;
                }
            }
            else {
                ghostPosition.linearSpeed = 0;
                ghostPosition.angularSpeed = 0;
            }
            break;

        case ROTATING:
            // ** 1. Couper la vitesse linéaire **
            ghostPosition.linearSpeed = 0;

            // ** 2. Gérer la Vitesse Angulaire **
            if (Abs(angleDiff) > angularStopDist) {
                double targetAng = (angleDiff > 0) ? maxAngularSpeed : -maxAngularSpeed;
                ghostPosition.angularSpeed = rampSpeed(ghostPosition.angularSpeed, targetAng, angularAccel, dt);
            } else {
                // Freinage
                ghostPosition.angularSpeed = rampSpeed(ghostPosition.angularSpeed, 0, angularAccel, dt);
            }

            // ** 3. L'angle fantôme est mis à jour par l'odométrie au début de la fonction. **

            // ** 4. Condition de fin de rotation standard **
            if (Abs(angleDiff) < ANGLE_TOLERANCE && Abs(ghostPosition.angularSpeed) < 0.1) {
                
                ghostPosition.angularSpeed = 0;
                
                // Transition : ROTATING -> ADVANCING
                currentState = ADVANCING;
                    
                // RESET PID
                robotState.PidTheta.erreurIntegrale = 0;
                robotState.PidX.erreurIntegrale = 0;
            }
            break;

        case LASTROTATE:
            // ** 1. Couper la vitesse linéaire (Garantie) **
            ghostPosition.linearSpeed = 0;

            // ** 2. Gérer la Vitesse Angulaire **
            if (Abs(angleDiff) > angularStopDist) {
                double targetAng = (angleDiff > 0) ? maxAngularSpeed : -maxAngularSpeed;
                ghostPosition.angularSpeed = rampSpeed(ghostPosition.angularSpeed, targetAng, angularAccel, dt);
            } else {
                // Freinage
                ghostPosition.angularSpeed = rampSpeed(ghostPosition.angularSpeed, 0, angularAccel, dt);
            }

            // ** 3. L'angle fantôme est mis à jour par l'odométrie au début de la fonction. **

            // ** 4. Condition de fin de rotation finale **
            if (Abs(angleDiff) < ANGLE_TOLERANCE && Abs(ghostPosition.angularSpeed) < 0.1) {
                ghostPosition.angularSpeed = 0;

                // Transition : LASTROTATE -> IDLE (Arrêt définitif du mouvement)
                currentState = IDLE;

                // RESET PID
                robotState.PidTheta.erreurIntegrale = 0;
                robotState.PidX.erreurIntegrale = 0;
            }
            break;

        case ADVANCING:
            
            // --- GESTION DE L'ARRÊT ET DE LA TRANSITION ---
            if (ghostPosition.distanceToTarget <= DISTANCE_TOLERANCE) {
                // ARRÊT FINAL ATTEINT
                ghostPosition.linearSpeed = 0;
                // Pas besoin de caler ghostPosition.x/y car ils sont mis à jour par l'odométrie
                currentState = IDLE; // IDLE va charger le prochain WP et initier la rotation
                break;
            }
            
            // --- VÉRIFICATION CONSTANTE DE L'ALIGNEMENT ---
            if (Abs(angleDiff) > ALIGNMENT_TOLERANCE) {
                // Si on dévie trop : Freinage et retour à la rotation
                ghostPosition.linearSpeed = rampSpeed(ghostPosition.linearSpeed, 0, linearAccel*2, dt);
                if (ghostPosition.linearSpeed < 0.01) {
                    currentState = ROTATING;
                }
            } else {
                // On est aligné : gestion de la vitesse linéaire
                if (ghostPosition.distanceToTarget > linearStopDist) {
                    ghostPosition.linearSpeed = rampSpeed(ghostPosition.linearSpeed, maxLinearSpeed, linearAccel, dt);
                } else {
                    // Freinage de décélération
                    ghostPosition.linearSpeed = rampSpeed(ghostPosition.linearSpeed, 0, linearAccel, dt);
                }
            }

            // --- CRITIQUE : Suppression de l'intégration position/angle ---
            // Ces lignes sont redondantes et fausses, car la position réelle
            // (robotState.xPosFromOdometry, etc.) est calculée dans QEI.c.
            /*
            ghostPosition.x += ghostPosition.linearSpeed * cos(ghostPosition.theta) * dt;
            ghostPosition.y += ghostPosition.linearSpeed * sin(ghostPosition.theta) * dt;
            */
            
            break;
    }
    
    // 4. Envoi des consignes au bas-niveau
    robotState.consigneVitesseLineaire = ghostPosition.linearSpeed;
    robotState.consigneVitesseAngulaire = ghostPosition.angularSpeed;
    
    ghostPosition.state = currentState;
}

/* ==================== ENVOI DES DONNÉES ==================== */

void SendGhostData(void) {
    unsigned char ghostPayload[32];
    
    getBytesFromInt32(ghostPayload, 0, timestamp);
    getBytesFromFloat(ghostPayload, 4, (float)ghostPosition.angleToTarget);
    getBytesFromFloat(ghostPayload, 8, (float)ghostPosition.distanceToTarget);
    getBytesFromFloat(ghostPayload, 12, (float)ghostPosition.theta);
    getBytesFromFloat(ghostPayload, 16, (float)ghostPosition.angularSpeed);
    getBytesFromFloat(ghostPayload, 20, (float)ghostPosition.x);
    getBytesFromFloat(ghostPayload, 24, (float)ghostPosition.y);
    getBytesFromFloat(ghostPayload, 28, (float)ghostPosition.linearSpeed);
    
    UartEncodeAndSendMessage(0x0050, 32, ghostPayload);
}

/* ==================== ACCESSEURS ==================== */

TrajectoryState GetCurrentTrajectoryState(void) {
    return currentState;
}

int GetCurrentWaypointIndex(void) {
    return waypointIndex;
}

void ResetTrajectory(void) {
    InitTrajectoryGenerator();
}