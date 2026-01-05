#include <xc.h>
#include "IO.h"
#include "PWM.h"
#include "robot.h"
#include "ToolBox.h"
#include "timer.h"
#include "QEI.h"
#include "UART.h"
#include "UART_Protocol.h"
#include "main.h"

// Variables PID
float erreur_prec = 0;
float somme_erreur = 0;

// PID constants
#define Kp  0.8
#define Ki  0.0
#define Kd  0.1
#define VITESSE_MAX 50.0

void ControlRobotArUco(void) {
    if (UartProtocol_ArUcoDetected()) {
        float x_aruco = UartProtocol_GetArUcoX();
        float erreur = x_aruco;
        somme_erreur += erreur;
        float variation = erreur - erreur_prec;
        erreur_prec = erreur;

        float correction = Kp*erreur + Ki*somme_erreur + Kd*variation;

        float vitesseGauche  = VITESSE_MAX - correction;
        float vitesseDroite  = VITESSE_MAX + correction;

        // Clamp
        if(vitesseGauche>VITESSE_MAX)vitesseGauche=VITESSE_MAX;
        if(vitesseGauche<-VITESSE_MAX)vitesseGauche=-VITESSE_MAX;
        if(vitesseDroite>VITESSE_MAX)vitesseDroite=VITESSE_MAX;
        if(vitesseDroite<-VITESSE_MAX)vitesseDroite=-VITESSE_MAX;

        PWMSetSpeedConsignePolaire((vitesseGauche+vitesseDroite)/2,
                                   (vitesseDroite-vitesseGauche)/DISTROUES);
    } else {
        PWMSetSpeedConsignePolaire(0,0);
        somme_erreur=0;
        erreur_prec=0;
    }
}

int main(void) {
    InitPWM();
    InitQEI();
    InitTimer();
    InitUART1();
    UART_Protocol_Init();

    while(1){
        UART_Protocol_Task();
        ControlRobotArUco();
    }
    return 0;
}
