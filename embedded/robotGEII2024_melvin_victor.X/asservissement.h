/* * File:   asservissement.h
 * Author: E306_PC2
 *
 * Created on 5 mai 2025, 16:21
 * Version : Corrigée pour intégration PID autonome
 */

#ifndef ASSERVISSEMENT_H
#define	ASSERVISSEMENT_H

#ifdef	__cplusplus
extern "C" {
#endif

    // Structure du correcteur PID
    typedef struct _PidCorrector {
        double Kp;
        double Ki;
        double Kd;
        double erreurProportionelleMax; // Saturation P
        double erreurIntegraleMax;      // Saturation I (Anti-windup)
        double erreurDeriveeMax;        // Saturation D
        
        // Mémoires (ne pas modifier manuellement)
        double erreurIntegrale;
        double epsilon_1; // Erreur précédente
        double erreur;
        
        // Valeurs de sortie pour debug
        double corrP;
        double corrI;
        double corrD;
    } PidCorrector;

    /* ==========================================
       PROTOTYPES DES FONCTIONS
       ========================================== */

    // ---> FONCTION AJOUTÉE (Indispensable pour charger les PID au démarrage)
    void InitAsservissement(void);

    // Fonctions de configuration et calcul
    void SetupPidAsservissement(volatile PidCorrector* PidCorr, double Kp, double Ki, double Kd, double proportionelleMax, double integralMax, double deriveeMax);
    void UpdateAsservissement(void);
    double Correcteur(volatile PidCorrector* PidCorr, double erreur);
    
    // Utilitaires mathématiques
    float LimitToIntervalBis(float value, float lowLimit, float highLimit);

    // Fonctions de communication / Debug (UART)
    void sendPidDonnees(void);
    void SendPidX(void);
    void SendPidTheta(void);
    void SendCommandeErreur(void);

#ifdef	__cplusplus
}
#endif

#endif	/* ASSERVISSEMENT_H */