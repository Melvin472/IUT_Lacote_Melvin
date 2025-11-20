#ifndef GHOSTMANAGER_H
#define GHOSTMANAGER_H

// Macros à garder (celles qui sont censées être dans ce fichier)
#define GHOST_DATA 0x0062 // J'ai gardé 0x0062 car c'était la première définition

// Tolérances pour considérer qu'on a atteint l'objectif
#define ANGLE_TOLERANCE      0.15  // ~2.8 degrés (Tolérance d'angle pour arrêter de tourner)
#define ALIGNMENT_TOLERANCE  0.20  // ~5.7 degrés (Tolérance pour oser avancer)
#define DISTANCE_TOLERANCE   0.1 // 5 mm (Tolérance de position)

// *** NOUVELLE DÉFINITION DE LA STRUCTURE ***
// Ceci résout l'erreur 'unknown type name 'Waypoint_t''
typedef struct {
    double x;
    double y;
    int isLastRotation;
} Waypoint_t;

// Enumération des états
typedef enum {
    IDLE = 0,
    ROTATING = 1,
    ADVANCING = 2,
    LASTROTATE = 3
} TrajectoryState;

// Structure de l'état du "fantôme" (vitesse et position de consigne)
typedef struct {
    double x;
    double y;
    double theta;
    double linearSpeed;
    double angularSpeed;
    double targetX;
    double targetY;
    double angleToTarget;
    double distanceToTarget;
    TrajectoryState state;
} GhostPosition;

// Prototypes
void InitTrajectoryGenerator(void);
void UpdateTrajectory(void);
void SendGhostData(void);
TrajectoryState GetCurrentTrajectoryState(void);
int GetCurrentWaypointIndex(void);
void ResetTrajectory(void);

#endif /* GHOSTMANAGER_H */