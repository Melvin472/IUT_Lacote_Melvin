/* 
 * File:   PWM.h
 * Author: GEII Robot
 *
 * Created on 18 septembre 2024, 13:43
 */

#ifndef PWM_H
#define	PWM_H
#define MOTEUR_DROIT 1
#define MOTEUR_GAUCHE 0

void InitPWM(void);
void PWMSetSpeedConsigne(float vitesseEnPourcents, char moteur);
void PWMUpdateSpeed();

#endif	/* PWM_H */

