#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "math.h"
#include "robot.h"   // Pour robotState, consignes, etc.

// === Définitions des constantes de trajectoire ===
#define GHOST_DATA             0x0050   // ID du message ghost
#define MAX_LINEAR_SPEED       1.0
#define MAX_LINEAR_ACCEL       0.2
#define MAX_ANGULAR_SPEED      (2.0 * PI)
#define MAX_ANGULAR_ACCEL      (2.0 * PI)
#define ANGLE_TOLERANCE        0.05
#define DISTANCE_TOLERANCE     0.1

// === États de trajectoire ===
typedef enum {
    IDLE,
    ROTATING,
    ADVANCING,
    LASTROTATE,
        GO_TO_POINT   
} TrajectoryState;

// === Structure du ghost (position virtuelle) ===
typedef struct {
    TrajectoryState state;
    double x;
    double y;
    double theta;
    double linearSpeed;
    double angularSpeed;
    double targetX;
    double targetY;
    double angleToTarget;
    double distanceToTarget;
    double red_target_x;
    double red_target_y;
} GhostPosition;

// === Variables globales ===
extern volatile GhostPosition ghostPosition;
extern int current_state;   // utilisé dans trajectory.c et uart_protocol.c

// === Fonctions exportées ===
void InitTrajectoryGenerator(void);
void UpdateTrajectory(void);
void SendGhostData(void);
void rotationTarget(double currentTime);

#endif // TRAJECTORY_H
