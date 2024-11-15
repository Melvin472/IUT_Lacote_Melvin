#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
#include "ChipConfig.h"
#include "IO.h"
#include "timer.h"
#include "PWM.h"
#include "Robot.h"
#include "ADC.h"
#include "main.h"
#include "ToolBox.h"
#include "UART.h"
#include <libpic30.h>
#include "CB_RX1.h"
#include "CB_TX1.h"

unsigned char stateRobot = STATE_ATTENTE_BP;

int main(void) {
    InitOscillator();
    InitIO();
    InitTimer23();
    InitTimer1();
    InitPWM();
    InitADC1();
    InitTimer4();
    InitUART();

    cbRx1Head = 0;
    cbRx1Tail = 0;

    PWMSetSpeedConsigne(20, 1);
    PWMSetSpeedConsigne(20, 2);

    while (1) {
        int i;
        for (i = 0; i < CB_RX1_GetDataSize(); i++) {
            unsigned char c = CB_RX1_Get();
            SendMessage(&c, 1);
        }

        __delay32(1000);

        if (ADCIsConversionFinished() == 1) {
            ADCClearConversionFinishedFlag();
            unsigned int *result = ADCGetResult();
            float volts = ((float) result[0]) * 3.3 / 4096;
            robotState.distanceTelemetreGauche = 34 / volts - 5;
            volts = ((float) result[1]) * 3.3 / 4096;
            robotState.distanceTelemetreCentre = 34 / volts - 5;
            volts = ((float) result[2]) * 3.3 / 4096;
            robotState.distanceTelemetreDroit = 34 / volts - 5;
            volts = ((float) result[3]) * 3.3 / 4096;
            robotState.distanceTelemetreExDroit = 34 / volts - 5;
            volts = ((float) result[4]) * 3.3 / 4096;
            robotState.distanceTelemetreExGauche = 34 / volts - 5;
        }

        if (robotState.distanceTelemetreDroit > 35 &&
            robotState.distanceTelemetreCentre > 25 &&
            robotState.distanceTelemetreGauche > 35) {
            LED_BLANCHE_2 = 1;
            LED_BLEUE_2 = 1;
            LED_ORANGE_2 = 1;
            LED_ROUGE_2 = 1;
            LED_VERTE_2 = 1;
        } else {
            LED_BLANCHE_2 = 0;
            LED_BLEUE_2 = 0;
            LED_ORANGE_2 = 0;
            LED_ROUGE_2 = 0;
            LED_VERTE_2 = 0;
        }

        if (BP2 != 0)
            stateRobot = STATE_ATTENTE;
    }
}

void OperatingSystemLoop(void) {
    switch (stateRobot) {
        case STATE_ATTENTE_BP:
            PWMSetSpeedConsigne(0, MOTEUR_DROIT);
            PWMSetSpeedConsigne(0, MOTEUR_GAUCHE);
            break;

        case STATE_ATTENTE:
            timestamp = 0;
            PWMSetSpeedConsigne(0, MOTEUR_DROIT);
            PWMSetSpeedConsigne(0, MOTEUR_GAUCHE);
            stateRobot = STATE_ATTENTE_EN_COURS;
            break;

        case STATE_ATTENTE_EN_COURS:
            if (timestamp > 1000)
                stateRobot = STATE_AVANCE;
            break;

        case STATE_AVANCE:
            PWMSetSpeedConsigne(24, MOTEUR_DROIT);
            PWMSetSpeedConsigne(23, MOTEUR_GAUCHE);
            stateRobot = STATE_AVANCE_EN_COURS;
            break;

        case STATE_AVANCE_EN_COURS:
            SetNextRobotStateInAutomaticMode();
            break;

        case STATE_TOURNE_GAUCHE:
            PWMSetSpeedConsigne(20, MOTEUR_DROIT);
            PWMSetSpeedConsigne(0, MOTEUR_GAUCHE);
            stateRobot = STATE_TOURNE_GAUCHE_EN_COURS;
            break;

        case STATE_TOURNE_GAUCHE_EN_COURS:
            SetNextRobotStateInAutomaticMode();
            break;

        case STATE_TOURNE_DROITE:
            PWMSetSpeedConsigne(0, MOTEUR_DROIT);
            PWMSetSpeedConsigne(20, MOTEUR_GAUCHE);
            stateRobot = STATE_TOURNE_DROITE_EN_COURS;
            break;

        case STATE_TOURNE_DROITE_EN_COURS:
            SetNextRobotStateInAutomaticMode();
            break;

        case STATE_TOURNE_SUR_PLACE_GAUCHE:
            PWMSetSpeedConsigne(15, MOTEUR_DROIT);
            PWMSetSpeedConsigne(-15, MOTEUR_GAUCHE);
            stateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE_EN_COURS;
            break;

        case STATE_TOURNE_SUR_PLACE_GAUCHE_EN_COURS:
            SetNextRobotStateInAutomaticMode();
            break;

        case STATE_TOURNE_SUR_PLACE_DROITE:
            PWMSetSpeedConsigne(-15, MOTEUR_DROIT);
            PWMSetSpeedConsigne(15, MOTEUR_GAUCHE);
            stateRobot = STATE_TOURNE_SUR_PLACE_DROITE_EN_COURS;
            break;

        case STATE_TOURNE_SUR_PLACE_DROITE_EN_COURS:
            SetNextRobotStateInAutomaticMode();
            break;

        default:
            stateRobot = STATE_ATTENTE;
            break;

        case STATE_ARRET:
            PWMSetSpeedConsigne(0, MOTEUR_DROIT);
            PWMSetSpeedConsigne(0, MOTEUR_GAUCHE);
            stateRobot = STATE_ARRET_EN_COURS;
            break;

        case STATE_ARRET_EN_COURS:
            break;
    }
}

unsigned char nextStateRobot = 0;

void SetNextRobotStateInAutomaticMode() {
    unsigned char positionObstacle = PAS_D_OBSTACLE;

    if (robotState.distanceTelemetreDroit < 35 && robotState.distanceTelemetreCentre > 25 && robotState.distanceTelemetreGauche > 35) 
        positionObstacle = OBSTACLE_A_DROITE;
    else if (robotState.distanceTelemetreDroit > 35 && robotState.distanceTelemetreCentre > 25 && robotState.distanceTelemetreGauche < 35) 
        positionObstacle = OBSTACLE_A_GAUCHE;
    else if (robotState.distanceTelemetreCentre < 25) 
        positionObstacle = OBSTACLE_EN_FACE;
    else if (robotState.distanceTelemetreDroit > 35 && robotState.distanceTelemetreCentre > 25 && robotState.distanceTelemetreGauche > 35) 
        positionObstacle = PAS_D_OBSTACLE;
    else if (robotState.distanceTelemetreExDroit < 35) 
        positionObstacle = OBSTACLE_EN_FACE;
    else if (robotState.distanceTelemetreExGauche < 35)
        positionObstacle = OBSTACLE_EN_FACE;

    if (positionObstacle == PAS_D_OBSTACLE)
        nextStateRobot = STATE_AVANCE;
    else if (positionObstacle == OBSTACLE_A_DROITE)
        nextStateRobot = STATE_TOURNE_GAUCHE;
    else if (positionObstacle == OBSTACLE_A_GAUCHE)
        nextStateRobot = STATE_TOURNE_DROITE;
    else if (positionObstacle == OBSTACLE_EN_FACE)
        nextStateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE;

    if (nextStateRobot != stateRobot - 1)
        stateRobot = nextStateRobot;
}
