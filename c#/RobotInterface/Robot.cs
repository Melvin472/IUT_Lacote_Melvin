using System;
using System.ComponentModel;
using System.Diagnostics;
using System.Windows;


public class Robot
{
    public Queue<byte> byteListReceived = new Queue<byte>();
    
    public float distanceTelemetreDroit;
    public float distanceTelemetreCentre;
    public float distanceTelemetreGauche;

    public float timestamp = 0.0f;
    public float xPosFromOdometry = 0.0f;
    public float yPosFromOdometry = 0.0f;
    public float angleRadianFromOdometry = 0.0f;
    public float vitesseLineaireFromOdometry = 0.0f;
    public float vitesseAngulaireFromOdometry = 0.0f;
    public Robot()
    {

    }
    public enum StateReception
    {
        Waiting,
        FunctionMSB,
        FunctionLSB,
        PayloadLengthMSB,
        PayloadLengthLSB,
        Payload,
        CheckSum
    }

    StateReception rcvState = StateReception.Waiting;

    int msgDecodedFunction = 0;
    int msgDecodedPayloadLength = 0;
    byte[] msgDecodedPayload = new byte[1];
    int msgDecodedPayloadIndex = 0;
    public int[] sensor = new int[5];

    public int[] getSensorValues() { return sensor; }

    public void CallDecodeMessage(byte c)
    {
        DecodeMessage(c);
    }

    private void SensorsPutData(byte[] payload)
    {
        for (int i = 0; i < payload.Length; i++)
        {
            sensor[i] = (int)payload[i];
        }
    }
    private void PositionReconstructData(byte[] c)
    {   
            timestamp = BitConverter.ToSingle(c, 0);
            xPosFromOdometry = BitConverter.ToSingle(c, 4);
            yPosFromOdometry = BitConverter.ToSingle(c, 8);
            angleRadianFromOdometry = BitConverter.ToSingle(c, 12);
            vitesseLineaireFromOdometry = BitConverter.ToSingle(c, 16);
            vitesseAngulaireFromOdometry = BitConverter.ToSingle(c, 20);
            Debug.WriteLine(timestamp);
    }
    private void DecodeMessage(byte c)
    {
        switch (rcvState)
        {
            case StateReception.Waiting:
                rcvState = StateReception.Waiting;
                if (c == 0XFE)
                    rcvState = StateReception.FunctionMSB;
                break;
            case StateReception.FunctionMSB:
                msgDecodedFunction = (c << 8);
                rcvState = StateReception.FunctionLSB;
                break;
            case StateReception.FunctionLSB:
                msgDecodedPayloadLength = 0;
                msgDecodedFunction |= c;
                rcvState = StateReception.PayloadLengthMSB;
                break;
            case StateReception.PayloadLengthMSB:
                msgDecodedPayloadLength = (c << 8);
                rcvState = StateReception.PayloadLengthLSB;
                break;
            case StateReception.PayloadLengthLSB:
                msgDecodedPayloadLength |= c;
                if (msgDecodedPayloadLength > 0)
                {
                    if(msgDecodedPayloadLength < 100) { //limit de la taille de payload
                        msgDecodedPayload = new byte[msgDecodedPayloadLength]; 
                        rcvState = StateReception.Payload;}
                    else { rcvState = StateReception.Waiting; }
                    
                }
                else
                {
                    rcvState = StateReception.CheckSum;
                }
                break;
            case StateReception.Payload:
                msgDecodedPayload[msgDecodedPayloadIndex++] = c;
                if (msgDecodedPayloadIndex >= msgDecodedPayloadLength)
                {
                    rcvState = StateReception.CheckSum;
                    msgDecodedPayloadIndex = 0;
                }
                break;
            case StateReception.CheckSum:
                byte res = CalculateChecksum(msgDecodedFunction, msgDecodedPayloadLength, msgDecodedPayload);
                if (res == c)
                {
                    switch (msgDecodedFunction)
                    {
                        case 0x0061:
                            if(msgDecodedPayloadLength == 24) {PositionReconstructData(msgDecodedPayload); }
                            
                            break;
                        case 0x0030:
                            SensorsPutData(msgDecodedPayload);
                            break;

                        default:
                            throw new Exception("Code non reconnu");
                    }
                    rcvState = StateReception.Waiting;
                }
                else
                {
                    rcvState = StateReception.Waiting;
                    Debug.WriteLine("CRC ERROR");
                }
                
                break;
            default:
                rcvState = StateReception.Waiting;
                break;
        }
    }
    byte CalculateChecksum(int msgFunction, int msgPayloadLength, byte[] msgPayload)
    {
        byte checksum = 0;
        checksum ^= 0xFE;
        checksum ^= (byte)(msgFunction >> 8);
        checksum ^= (byte)(msgFunction >> 0);
        checksum ^= (byte)(msgPayloadLength >> 8);
        checksum ^= (byte)(msgPayloadLength >> 0);


        for (int i = 0; i < msgPayload.Length; i++)
        {
            checksum ^= msgPayload[i];
        }
        return checksum;

    }


   
}