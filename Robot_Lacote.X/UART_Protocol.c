
#include "xc.h"
#include "UART_Protocol.h"
#include "IO.h"
#include "CB_TX1.H"
unsigned char UartCalculateChecksum(int msgFunction, int msgPayloadLength, unsigned char* msgPayload) {
    unsigned char checksum;
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
    SendMessage(tram, 6 + msgPayloadLength);
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


char CalculateChecksum(int msgFunction, int msgPayloadLength, unsigned char* msgPayload) {
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
            SendMessage(&CHK,1);
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
