
#include "xc.h"
#include "UART_Protocol.h"
#include "IO.h"
#include "CB_TX1.h"
#include "Robot.h"
#include "timer.h"

int msgDecodedFunction = 0;
int msgDecodedPayloadLenght = 0;
unsigned char msgDecodedPayload[128];
int msgDecodedPayloadIndex = 0;
int rcvState = 0;
unsigned char calculatedChecksum ;


unsigned char UartCalculateChecksum(int msgFunction, int msgPayloadLength, unsigned char* msgPayload) {
    unsigned char checksum = 0;
    
    checksum ^= 0xFE;
    checksum ^= 0x00;
    checksum ^= (unsigned char) msgFunction;
    checksum ^= (unsigned char) msgPayloadLength;

    for (int i = 0; i < msgPayloadLength; i++) {
        checksum ^= msgPayload[i];
    }
    return checksum;
}

void UartEncodeAndSendMessage(int msgFunction, int msgPayloadLength, unsigned char* msgPayload) {

    unsigned char tram[msgPayloadLength + 6];
    
    tram[0] = 0xFE;
    tram[1] = 0x00;
    tram[2] = (unsigned char) msgFunction;
    tram[3] = (unsigned char) (msgPayloadLength>>8);
    tram[4] = (unsigned char) (msgPayloadLength & 0xFF);

    for (int i = 0; i < msgPayloadLength; i++) {
        tram[i + 5] = msgPayload[i];
    }
    
    trame[msgPayloadLength +5] = UartCalculateChecksum(msgFunction, msgPayloadLength, msgPayload);
    SendMessage(trame, msgPayloadLength +6);
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
            rcvState = PayloadLengthMSB;
            break;
        case PayloadLengthMSB:
            msgDecodedPayloadLength = (c << 8);
            rcvState = PayloadLengthLSB;
            break;
        case PayloadLengthLSB:
            msgDecodedPayloadLength |= c;
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
            char CHK = CalculateChecksum(msgDecodedFunction, msgDecodedPayloadLength, msgDecodedPayload);
              if (CHK == c) {
                rcvState = Waiting;
                UartProcessDecodedMessage(msgDecodedFunction, msgDecodedPayloadLength, msgDecodedPayload);
            }

            break;
        
            
              
    }
}


void UartProcessDecodedMessage(int function, int payloadLength, unsigned char* payload)
{
//Fonction appelee apres le decodage pour executer l?action
//correspondant au message recu
    //UartEncodeAndSendMessage(function,payloadLength,payload);
    if(function == 0x0020)
    {
        LED_BLANCHE_1 = 1;
    }
    if(function == 0x0080)
    {
        LED_BLANCHE_1 = 0;
    }


}
