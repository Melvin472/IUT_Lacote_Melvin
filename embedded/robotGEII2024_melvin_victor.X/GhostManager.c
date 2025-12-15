/*
 * File:   GhostManager.c
 * Gestion des waypoints et trajectoire du robot
 * Version : Logique du Carré Corrigée avec Sécurité Overshoot
 */

#include <math.h>
#include "GhostManager.h"
#include "Robot.h"      // Contient la structure ROBOT_STATE_BITS
#include "utilities.h"  // Contient Min/Max/Abs et PI
#include "UART_Protocol.h"
#include <xc.h>

// --- CORRECTION : Définition de PI si manquant ---
#ifndef PI
#define PI 3.14159265358979323846
#endif

// --- CORRECTION : Paramètres de Tolérance Explicites ---
// Ces valeurs sont critiques. Si elles sont trop petites, le robot ne s'arrête jamais.
#ifndef DISTANCE_TOLERANCE
#define DISTANCE_TOLERANCE 0.03  // 3 cm (Si on est à moins de 3cm, on valide)
#endif

#ifndef ANGLE_TOLERANCE
#define ANGLE_TOLERANCE 0.05     // ~3 degrés (Pour finir la rotation sur place)
#endif

#ifndef ALIGNMENT_TOLERANCE
#define ALIGNMENT_TOLERANCE 0.3  // ~17 degrés (Tolérance en ligne droite avant de corriger)
#endif

extern unsigned long timestamp;
extern volatile ROBOT_STATE_BITS robotState;

volatile GhostPosition ghostPosition; // Représente la CONSIGNE de trajectoire

/* ==================== PARAMÈTRES DE CONTRÔLE ==================== */
double maxAngularSpeed = 0.1;       // rad/s
double angularAccel = 0.1;          // rad/s^2
double maxLinearSpeed = 0.1;       // m/s (Légèrement augmenté)
double linearAccel = 0.1;          // m/s^2

/* ==================== GESTION DES WAYPOINTS ==================== */
#define MAX_WAYPOINTS 4

// Trajectoire : Carré de 20cm x 20cm
// CORRECTION : J'ai réactivé les 4 points pour tester la séquence complète
Waypoint_t waypoints[MAX_WAYPOINTS] = {
    {0.2,  0.0,  0},  // W1 : Coin 1
    {0.2,  0.2,  0},  // W2 : Coin 2
    {0.0,  0.2,  0},  // W3 : Coin 3
    {0.0,  0.0,  0}   // W4 : Retour
};

static int waypointIndex = 0;
static TrajectoryState currentState = IDLE;

/* ==================== INITIALISATION ==================== */
void InitTrajectoryGenerator(void) {
    ghostPosition.x = 0.0;
    ghostPosition.y = 0.0;
    ghostPosition.theta = 0.0;
    ghostPosition.linearSpeed = 0.0;
    ghostPosition.angularSpeed = 0.0;
    ghostPosition.targetX = 0.0;
    ghostPosition.targetY = 0.0;
    ghostPosition.state = IDLE;
    
    // Reset des positions odométriques pour partir d'une base saine
    robotState.xPosFromOdometry = 0.0;
    robotState.yPosFromOdometry = 0.0;
    robotState.angleRadianFromOdometry = 0.0;

    waypointIndex = 0;
    currentState = IDLE;
}

/* ==================== FONCTIONS UTILITAIRES ==================== */

static double computeAngleDifference(double theta, double thetaTarget) {
    double diff = thetaTarget - theta;
    while (diff > PI) diff -= 2 * PI;
    while (diff < -PI) diff += 2 * PI;
    return diff;
}

static double rampSpeed(double currentSpeed, double targetSpeed, double accel, double dt) {
    double maxChange = accel * dt;
    if (currentSpeed < targetSpeed) {
        return Min(currentSpeed + maxChange, targetSpeed);
    } else if (currentSpeed > targetSpeed) {
        return Max(currentSpeed - maxChange, targetSpeed);
    }
    return currentSpeed;
}

/* ==================== MISE À JOUR DE LA TRAJECTOIRE ==================== */

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

    // Mise à jour de la position fantôme avec la position RÉELLE (Odométrie)
    ghostPosition.x = robotState.xPosFromOdometry;
    ghostPosition.y = robotState.yPosFromOdometry;
    ghostPosition.theta = robotState.angleRadianFromOdometry;
    
    // 2. Calculs Géométriques vers la cible actuelle
    double dx = ghostPosition.targetX - ghostPosition.x;
    double dy = ghostPosition.targetY - ghostPosition.y;

    double thetaTarget = atan2(dy, dx);

    if (currentState == LASTROTATE) {
         thetaTarget = 0.0; // Rotation finale (si nécessaire)
    }

    double angleDiff = computeAngleDifference(ghostPosition.theta, thetaTarget);

    ghostPosition.angleToTarget = angleDiff;
    ghostPosition.distanceToTarget = sqrt(dx * dx + dy * dy);

    // 3. Calcul des distances de freinage
    double angularStopDist = (ghostPosition.angularSpeed * ghostPosition.angularSpeed) / (2 * angularAccel);
    double linearStopDist = (ghostPosition.linearSpeed * ghostPosition.linearSpeed) / (2 * linearAccel);

    // ============ MACHINE À ÉTAT ============

    switch (currentState) {

        case IDLE:
            if (MAX_WAYPOINTS > 0) {
                
                if (waypointIndex >= MAX_WAYPOINTS) {
                    waypointIndex = 0; // Boucle infinie du carré
                }
                
                Waypoint_t nextWay = waypoints[waypointIndex++];
                ghostPosition.targetX = nextWay.x;
                ghostPosition.targetY = nextWay.y;
                
                // Recalcul immédiat des distances
                dx = ghostPosition.targetX - ghostPosition.x;
                dy = ghostPosition.targetY - ghostPosition.y;
                ghostPosition.distanceToTarget = sqrt(dx * dx + dy * dy);
                
                // Si le point est déjà atteint (trop proche), on reste en IDLE pour prendre le suivant au prochain cycle
                if (ghostPosition.distanceToTarget < DISTANCE_TOLERANCE) {
                    currentState = IDLE; 
                }
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
            // Rotation sur place
            ghostPosition.linearSpeed = 0;

            if (Abs(angleDiff) > angularStopDist) {
                double targetAng = (angleDiff > 0) ? maxAngularSpeed : -maxAngularSpeed;
                ghostPosition.angularSpeed = rampSpeed(ghostPosition.angularSpeed, targetAng, angularAccel, dt);
            } else {
                ghostPosition.angularSpeed = rampSpeed(ghostPosition.angularSpeed, 0, angularAccel, dt);
            }
            
            // Fin de rotation
            if (Abs(angleDiff) < ANGLE_TOLERANCE && Abs(ghostPosition.angularSpeed) < 0.1) {
                ghostPosition.angularSpeed = 0;
                currentState = ADVANCING;
                    
                robotState.PidTheta.erreurIntegrale = 0;
                robotState.PidX.erreurIntegrale = 0;
            }
            break;

        case LASTROTATE:
            ghostPosition.linearSpeed = 0;
            if (Abs(angleDiff) > angularStopDist) {
                double targetAng = (angleDiff > 0) ? maxAngularSpeed : -maxAngularSpeed;
                ghostPosition.angularSpeed = rampSpeed(ghostPosition.angularSpeed, targetAng, angularAccel, dt);
            } else {
                ghostPosition.angularSpeed = rampSpeed(ghostPosition.angularSpeed, 0, angularAccel, dt);
            }

            if (Abs(angleDiff) < ANGLE_TOLERANCE && Abs(ghostPosition.angularSpeed) < 0.1) {
                ghostPosition.angularSpeed = 0;
                currentState = IDLE;
                robotState.PidTheta.erreurIntegrale = 0;
                robotState.PidX.erreurIntegrale = 0;
            }
            break;

        case ADVANCING:
            
            // --- CORRECTION MAJEURE : SÉCURITÉS D'ARRÊT ---

            // 1. Condition Normale : On est dans le cercle de tolérance
            if (ghostPosition.distanceToTarget <= DISTANCE_TOLERANCE) {
                ghostPosition.linearSpeed = 0;
                currentState = IDLE; 
                break;
            }

            // 2. Condition de Sauvegarde (Overshoot) : 
            // Si l'angle vers la cible dépasse 90° (PI/2), c'est que le point est DERRIÈRE nous.
            // On arrête tout de suite pour ne pas continuer à l'infini.
            if (Abs(angleDiff) > (PI / 2.0)) {
                 ghostPosition.linearSpeed = 0;
                 currentState = IDLE;
                 break;
            }
            
            // --- GESTION DU MOUVEMENT ---

            // Si on dévie trop de la ligne droite, on freine pour corriger l'angle
            if (Abs(angleDiff) > ALIGNMENT_TOLERANCE) {
                // Freinage d'urgence linéaire
                ghostPosition.linearSpeed = rampSpeed(ghostPosition.linearSpeed, 0, linearAccel*3, dt);
                
                // Si on est presque arrêté, on repasse en mode rotation pure
                if (ghostPosition.linearSpeed < 0.01) {
                    currentState = ROTATING;
                }
            } else {
                // On est aligné : Gestion de la vitesse trapézoïdale
                if (ghostPosition.distanceToTarget > linearStopDist) {
                    // Accélération / Vitesse constante
                    ghostPosition.linearSpeed = rampSpeed(ghostPosition.linearSpeed, maxLinearSpeed, linearAccel, dt);
                } else {
                    // Décélération pour arriver à vitesse nulle
                    ghostPosition.linearSpeed = rampSpeed(ghostPosition.linearSpeed, 0, linearAccel, dt);
                }
            }
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