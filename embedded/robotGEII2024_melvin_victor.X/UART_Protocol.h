#include "asservissement.h"
#include "robot.h"


#ifndef UART_Protocol_H
#define	UART_Protocol_H

unsigned char UartCalculateChecksum(int msgFunction, int msgPayloadLength, unsigned char* msgPayload);
void UartEncodeAndSendMessage(int msgFunction, int msgPayloadLength, unsigned char* msgPayload);
void UartDecodeMessage(unsigned char c);
void UartProcessDecodedMessage(int function, int payloadLength, unsigned char* payload);
void EnvoieDistanceTelemetre();
void EvoieMoteurInfo();
void sendled();

#define PidXConf 0x0091  //linéaire
#define PidThetaConf 0x0092  //angulaire
#define PidThetaXConf 0x0093
#define GHOST_DATA 0x0050
#define LOCK_TARGET 0x0051
#define ARUCO_DATA_RX       0x00A1 // ID de la trame pour les données ArUco reçues de la caméra JEVOIS (Message 3D)
#define ARUCO_TARGET_LOCKED 0x00A2

typedef enum {
    Waiting,
    FunctionMSB,
    FunctionLSB,
    PayloadLengthMSB,
    PayloadLengthLSB,
    Payload,
    CheckSum
} StateReception;

#endif	/* UART_H */