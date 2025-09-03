#ifndef ASSERVISSEMENT_H
#define ASSERVISSEMENT_H

#define FREQ_ECH_QEI 100.0 // Fréquence d'échantillonnage en Hz

typedef struct _PidCorrector
{
    double Kp;
    double Ki;
    double Kd;
    double erreurProportionelleMax;
    double erreurIntegraleMax;
    double erreurDeriveeMax;
    double erreurIntegrale;
    double epsilon_1;
    double erreur;
    // For Debug only
    double corrP;
    double corrI;
    double corrD;
} PidCorrector;

// Fonction pour limiter une valeur dans un intervalle
double LimitToInterval(double value, double min, double max);

// Fonction de configuration du PID
void SetupPidAsservissement(volatile PidCorrector* PidCorr, double Kp, double Ki, double Kd, 
                           double proportionelleMax, double integralMax, double deriveeMax);

// Fonction du correcteur PID
double Correcteur(volatile PidCorrector* PidCorr, double erreur);

// Fonction de mise à jour de l'asservissement
void UpdateAsservissement();



#endif // ASSERVISSEMENT_H

