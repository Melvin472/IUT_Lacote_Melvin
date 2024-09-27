#include <xc.h>
#include "IO.h"
#include "PWM.h"
#include "Robot.h"
#include "ToolBox.h"
#define PWMPER 24.0

void InitPWM(void) {
    PTCON2bits.PCLKDIV = 0b000; //Divide by 1
    PTPER = 100 * PWMPER; //éPriode en pourcentage
    //éRglage PWM moteur 1 sur hacheur 1
    IOCON1bits.PMOD = 0b11; //PWM I/O pin pair is in the True Independent Output mode
    IOCON1bits.PENL = 1;
    IOCON1bits.PENH = 1;
    FCLCON1 = 0x0003; //éDsactive la gestion des faults
    IOCON2bits.PMOD = 0b11; //PWM I/O pin pair is in the True Independent Output mode
    IOCON2bits.PENL = 1;
    IOCON2bits.PENH = 1;
    FCLCON2 = 0x0003; //éDsactive la gestion des faults
    /* Enable PWM Module */
    PTCONbits.PTEN = 1;
}
double talon = 50;
float acceleration = 5;

void PWMSetSpeedConsigne(float vitesseEnPourcents, char moteur) {
    if (moteur) {
        robotState.vitesseDroiteConsigne = -vitesseEnPourcents;
    } else {
        robotState.vitesseGaucheConsigne = vitesseEnPourcents;
    }
}
//    if (moteur==1){
//        if (vitesseEnPourcents > 0){
//        PDC1 = vitesseEnPourcents * PWMPER + talon;
//        SDC1 = talon;
//        }
//            else{
//                PDC1 = talon;
//                SDC1 = -vitesseEnPourcents * PWMPER + talon;
//            }
//    }
//    if (moteur==0){
//            if (vitesseEnPourcents > 0){
//            PDC2 = vitesseEnPourcents * PWMPER + talon;
//            SDC2 = talon;
//            }
//                else{
//                    PDC2 = talon;
//                    SDC2 = -vitesseEnPourcents * PWMPER + talon;
//                }

void PWMUpdateSpeed() {
    // Cette fonction est appelee sur timer et permet de suivre des rampes d acceleration
    if (robotState.vitesseDroiteCommandeCourante < robotState.vitesseDroiteConsigne)
        robotState.vitesseDroiteCommandeCourante = Min(
            robotState.vitesseDroiteCommandeCourante + acceleration,
            robotState.vitesseDroiteConsigne);
    if (robotState.vitesseDroiteCommandeCourante > robotState.vitesseDroiteConsigne)
        robotState.vitesseDroiteCommandeCourante = Max(
            robotState.vitesseDroiteCommandeCourante - acceleration,
            robotState.vitesseDroiteConsigne);
    if (robotState.vitesseDroiteCommandeCourante >= 0) {
        SDC1 = robotState.vitesseDroiteCommandeCourante * PWMPER + talon;
        PDC1 = talon;
    } else {
        SDC1 = talon;
        PDC1 = -robotState.vitesseDroiteCommandeCourante * PWMPER + talon;
    }
    if (robotState.vitesseGaucheCommandeCourante < robotState.vitesseGaucheConsigne)
        robotState.vitesseGaucheCommandeCourante = Min(
            robotState.vitesseGaucheCommandeCourante + acceleration,
            robotState.vitesseGaucheConsigne);
    if (robotState.vitesseGaucheCommandeCourante > robotState.vitesseGaucheConsigne)
        robotState.vitesseGaucheCommandeCourante = Max(
            robotState.vitesseGaucheCommandeCourante - acceleration,
            robotState.vitesseGaucheConsigne);
    if (robotState.vitesseGaucheCommandeCourante > 0) {
        SDC2 = robotState.vitesseGaucheCommandeCourante * PWMPER + talon;
        PDC2 = talon;
    } else {
        SDC2 = talon;
        PDC2 = -robotState.vitesseGaucheCommandeCourante * PWMPER + talon;
    }
}
