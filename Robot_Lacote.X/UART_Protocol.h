#ifndef XC_HEADER_TEMPLATE_H
#define	XC_HEADER_TEMPLATE_H

#define RCV_STATE_WAITING 0
#define RCV_STATE_FUNCTION_MSB 1
#define RCV_STATE_FUNCTION_LSB 2
#define RCV_STATE_LENGHT_MSB 3
#define RCV_STATE_LENGHT_LSB 4
#define RCV_STATE_PAYLOAD 5
#define RCV_STATE_CHECKSUM 6

#include <xc.h> 
#ifdef	__cplusplus
#endif

    void UartEncodeAndSendMessage(int msgFunction, int msgPayloadLength, unsigned char* msgPayload);
    void UartDecodeMessage(unsigned char c);
    char CalculateChecksum(int msgFunction, int msgPayloadLength, unsigned char* msgPayload);
    void UartProcessDecodedMessage(int function, int payloadLength, unsigned char* payload);
    
#ifdef	__cplusplus
}
#endif 
#endif	
