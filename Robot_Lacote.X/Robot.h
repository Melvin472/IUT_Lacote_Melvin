
#ifndef ROBOT_H
#define ROBOT_H



void PWMSetSpeedConsigne(float vitesseEnPourcents, char moteur);
void PWMUpdateSpeed();

typedef struct robotStateBITS {

    union {

        struct {
            unsigned char taskEnCours;
            float vitesseGaucheConsigne;
            float vitesseGaucheCommandeCourante;
            float vitesseDroiteConsigne;
            float vitesseDroiteCommandeCourante;
            float distanceTelemetreDroit;
            float distanceTelemetreCentre;
            float distanceTelemetreGauche;
            float distanceTelemetreExGauche;
            float distanceTelemetreExDroit;
           
            
            float vitesseDroitFromOdometry;
            float vitesseGaucheFromOdometry;
            float xPosFromOdometry_1;
            float yPosFromOdometry_1;
            float angleRadianFromOdometry_1;
            float xPosFromOdometry;
            float yPosFromOdometry;
            float angleRadianFromOdometry;
            float vitesseLineaireFromOdometry;
            float vitesseAngulaireFromOdometry;
            
            float correctionVitesseLineaire;
            float correctionVitesseAngulaire;
            
            float consigneVitesseLineaire;
            float consigneVitesseAngulaire;
        };
    };
} ROBOT_STATE_BITS;

extern volatile ROBOT_STATE_BITS robotState;


#endif

