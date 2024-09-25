/* 
 * File:   PWM.h
 * Author: GEII Robot
 *
 * Created on 18 septembre 2024, 13:43
 */

#ifndef PWM_H
#define	PWM_H
#define MOTEUR_DROIT 0
#define MOTEUR_GAUCHE 1

void InitPWM(void);
void PWMSetSpeedConsigne(float vitesseEnPourcents, char moteur);
void PWMUpdateSpeed();

#endif	/* PWM_H */

