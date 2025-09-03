#include "asservissement.h"
#include "robot.h"
#include "PWM.h" // Pour PWMSetSpeedConsignePolaireorco
#include <math.h>
#include "ToolBox.h"
// Fonction pour limiter une valeur dans un intervalle


// Configuration des paramètres PID
void SetupPidAsservissement(volatile PidCorrector* PidCorr, double Kp, double Ki, double Kd, 
                          double proportionelleMax, double integralMax, double deriveeMax)
{
    PidCorr->Kp = Kp;
    PidCorr->Ki = Ki;
    PidCorr->Kd = Kd;
    PidCorr->erreurProportionelleMax = proportionelleMax;
    PidCorr->erreurIntegraleMax = integralMax;
    PidCorr->erreurDeriveeMax = deriveeMax;
    PidCorr->erreurIntegrale = 0.0;
    PidCorr->epsilon_1 = 0.0;
    PidCorr->erreur = 0.0;
    PidCorr->corrP = 0.0;
    PidCorr->corrI = 0.0;
    PidCorr->corrD = 0.0;
}



// Fonction de correction PID
double Correcteur(volatile PidCorrector* PidCorr, double erreur)
{
    PidCorr->erreur = erreur;
    
    // Terme proportionnel
    double erreurProportionnelle = LimitToInterval(erreur, 
                                                -PidCorr->erreurProportionelleMax/PidCorr->Kp, 
                                                PidCorr->erreurProportionelleMax/PidCorr->Kp);
    PidCorr->corrP = erreurProportionnelle * PidCorr->Kp;
    
    // Terme intégral
    PidCorr->erreurIntegrale += erreur / FREQ_ECH_QEI;
    PidCorr->erreurIntegrale = LimitToInterval(PidCorr->erreurIntegrale, 
                                             -PidCorr->erreurIntegraleMax/PidCorr->Ki, 
                                             PidCorr->erreurIntegraleMax/PidCorr->Ki);
    PidCorr->corrI = PidCorr->erreurIntegrale * PidCorr->Ki;
    
    // Terme dérivé
    double erreurDerivee = (erreur - PidCorr->epsilon_1) * FREQ_ECH_QEI;
    double deriveeBornee = LimitToInterval(erreurDerivee, 
                                         -PidCorr->erreurDeriveeMax/PidCorr->Kd, 
                                         PidCorr->erreurDeriveeMax/PidCorr->Kd);
    PidCorr->epsilon_1 = erreur;
    PidCorr->corrD = deriveeBornee * PidCorr->Kd;
    
    return PidCorr->corrP + PidCorr->corrI + PidCorr->corrD;
}

// Mise à jour de l'asservissement
void UpdateAsservissement()
{
//    robotState.PidX.erreur = ;
//    robotState.PidTheta.erreur = ;
//
//    robotState.CorrectionVitesseLineaire =
//    Correcteur(&robotState.PidX, robotState.PidX.erreur);
//    robotState.CorrectionVitesseAngulaire = ;
//
//    PWMSetSpeedConsignePolaire(robotState.CorrectionVitesseLineaire,
//    robotState.CorrectionVitesseAngulaire);
}