/*
 * File:   UART_Protocol.c
 * Author: E306-PC3
 *
 * Created on 10 décembre 2024, 10:53
 */


#include "xc.h"
#include "CB_TX1.H"
#include "UART_Protocol.h"
#include "IO.h"
#include "CB_TX1.H"

unsigned char chk;

unsigned char UartCalculateChecksum(int msgFunction, int msgPayloadLength, unsigned char* msgPayload) {
    unsigned char checksum = 0;
    checksum ^= 0xFE;
    checksum ^= (msgFunction >> 8);
    checksum ^= (msgFunction >> 0);
    checksum ^= (msgPayloadLength >> 8);
    checksum ^= (msgPayloadLength >> 0);


    for (int i = 0; i < msgPayloadLength; i++) {
        checksum ^= msgPayload[i];
    }
    return checksum;
}

void UartEncodeAndSendMessage(int msgFunction, int msgPayloadLength, unsigned char* msgPayload) {

    unsigned char tram[6 + msgPayloadLength];
    tram[0] = 0xFE;
    tram[1] = (msgFunction >> 8);
    tram[2] = (msgFunction);
    tram[3] = (msgPayloadLength >> 8);
    tram[4] = (msgPayloadLength);

    for (int i = 5; i < msgPayloadLength + 5; i++) {
        tram[i] = (msgPayload[i - 5]);
    }
    char checkSum = UartCalculateChecksum(msgFunction, msgPayloadLength, msgPayload);
    tram[5 + msgPayloadLength] = checkSum;
    SendMessage((char*)tram, 6 + msgPayloadLength);
    }
int msgDecodedFunction = 0;
int msgDecodedPayloadLength = 0;
unsigned char msgDecodedPayload[128];
int msgDecodedPayloadIndex = 0;

enum StateReception {
    Waiting,
    FunctionMSB,
    FunctionLSB,
    PayloadLengthMSB,
    PayloadLengthLSB,
    Payload,
    CheckSum
} rcvState;



void UartProcessDecodedMessage(int function, int payloadLength, unsigned char* payload)
{
    if(function == 0x0020)
    {
        

        if((int)payload[1] == 0x31){
            if(payload[2] == 0x31){
                LED_BLEUE_2 = 1;
            }else if(payload[2] == 0x30){
                LED_BLEUE_2 = 0;
            }
        }else if((int)payload[1] == 0x32){
            if(payload[2] == 0x31){
                LED_ORANGE_2 = 1;
            }else if(payload[2] == 0x30){
                LED_ORANGE_2 = 0;
            }
        }else if((int)payload[1] == 0x33){
            if(payload[2] == 0x31){
                LED_ROUGE_2 = 1;
            }else if(payload[2] == 0x30){
                LED_ROUGE_2 = 0;
            }
        }else if((int)payload[1] == 0x34){//sex    
            if(payload[2] == 0x31){
                LED_VERTE_2 = 1;
            }else if(payload[2] == 0x30){
                LED_VERTE_2 = 0;
            }
        }else if((int)payload[1] == 0x30){
            if(payload[2] == 0x31){
                LED_BLANCHE_2 = 1;
            }else if(payload[2] == 0x30){
                LED_BLANCHE_2 = 0;
            }
        }
    }
    if(function == 0x0040)
    {
        if(payload[1] == 0x00)
        {
            //int speed = atoi(payload & 0x);
            
        }
        else if(payload[1] == 0x01){}
    }


}

void UartDecodeMessage(unsigned char c) {
    
    switch (rcvState) {
        case Waiting:
            rcvState = Waiting;
            if (c == 0XFE)
                rcvState = FunctionMSB;
            break;
        case FunctionMSB:
            msgDecodedFunction |= (c << 8);
            rcvState = FunctionLSB;
            break;
        case FunctionLSB:
            msgDecodedFunction |= c;
            msgDecodedFunction = msgDecodedFunction;
            rcvState = PayloadLengthMSB;
            break;
        case PayloadLengthMSB:
            msgDecodedPayloadLength = (c << 8);
            rcvState = PayloadLengthLSB;
            break;
        case PayloadLengthLSB:
            msgDecodedPayloadLength |= c;
            msgDecodedPayloadLength = msgDecodedPayloadLength;
            if (msgDecodedPayloadLength > 0) {
                
                rcvState = Payload;
            } else {
                rcvState = CheckSum;
            }
            break;
        case Payload:
            msgDecodedPayloadIndex += 1;
            msgDecodedPayload[msgDecodedPayloadIndex] = c;
            if (msgDecodedPayloadIndex >= msgDecodedPayloadLength){
                msgDecodedPayloadIndex = 0;
                rcvState = CheckSum;}
                
            break;
        case CheckSum:
            
            chk = UartCalculateChecksum(msgDecodedFunction, msgDecodedPayloadLength, msgDecodedPayload);
            if (chk == c) {
                
                rcvState = Waiting;
                
                UartProcessDecodedMessage(msgDecodedFunction, msgDecodedPayloadLength, msgDecodedPayload);
                msgDecodedFunction = 0;
                msgDecodedPayloadLength = 0;
                
          
            }

            break;
        
            
              
    }
}


