/* 
 * File:   Robot.h
 * Author: GEII Robot
 *
 * Created on 16 septembre 2024, 14:43
 */

#ifndef ROBOT_H
#define ROBOT_H
typedef struct robotStateBITS {
    union {
        struct {
            unsigned char taskEnCours;
            float vitesseGaucheConsigne;
            float vitesseGaucheCommandeCourante;
            float vitesseDroiteConsigne;
            float vitesseDroiteCommandeCourante;
        };
    };
} ROBOT_STATE_BITS;
extern volatile ROBOT_STATE_BITS robotState;
#endif /* ROBOT_H */

