using System;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Threading;
using ExtendedSerialPort_NS;
using System.IO.Ports;

namespace RobotInterface
{

    public partial class MainWindow : Window
    {
        private ExtendedSerialPort serialPort1;
        private DispatcherTimer timerAffichage;
        private Robot robot = new Robot();
        private Reception reception = new Reception();

        public MainWindow()
        {
            InitializeComponent();
            InitializeSerialPort();
            InitializeTimer();
        }

        private void InitializeSerialPort()
        {
            serialPort1 = new ExtendedSerialPort("COM4", 115200, Parity.None, 8, StopBits.One);
            serialPort1.DataReceived += 
                
                
                
                
        SerialPort1_DataReceived;
            serialPort1.Open();
        }

        private void InitializeTimer()
        {
            timerAffichage = new DispatcherTimer
            {
                Interval = TimeSpan.FromMilliseconds(1)
            };
            timerAffichage.Tick += TimerAffichage_Tick;
            timerAffichage.Start();
        }

        private void TimerAffichage_Tick(object sender, EventArgs e)
        {
            while (robot.byteListReceived.Count > 0)
            {
                byte receivedByte = robot.byteListReceived.Dequeue();
                textReceivedAdd(receivedByte.ToString("X2"));
                reception.CallDecodeMessage(receivedByte);  
            }
        }

        public void textReceivedAdd(string textReceived)
        {
            textboxReception.Text += textReceived;
        }

        private void UartEncodeAndSendMessage(int msgFunction, int msgPayloadLength, byte[] msgPayload)
        {
            byte[] tram = new byte[6 + msgPayloadLength];
            tram[0] = 0xFE;
            tram[1] = (byte)(msgFunction >> 8);
            tram[2] = (byte)(msgFunction);
            tram[3] = (byte)(msgPayloadLength >> 8);
            tram[4] = (byte)(msgPayloadLength);

            Array.Copy(msgPayload, 0, tram, 5, msgPayload.Length);

            byte checkSum = CalculateChecksum(msgFunction, msgPayloadLength, msgPayload);
            tram[5 + msgPayloadLength] = checkSum;
            serialPort1.Write(tram, 0, tram.Length);
        }

        private byte CalculateChecksum(int msgFunction, int msgPayloadLength, byte[] msgPayload)
        {
            byte checksum = 0;
            checksum ^= 0xFE;
            checksum ^= (byte)(msgFunction >> 8);
            checksum ^= (byte)(msgFunction);
            checksum ^= (byte)(msgPayloadLength >> 8);
            checksum ^= (byte)(msgPayloadLength);

            foreach (byte b in msgPayload)
            {
                checksum ^= b;
            }

            return checksum;
        }

        private void sendMessage()
        {
            serialPort1.Write(textboxEmission.Text);
        }

        private void SerialPort1_DataReceived(object sender, DataReceivedArgs e)
        {
            foreach (byte dataByte in e.Data)
            {
                robot.byteListReceived.Enqueue(dataByte);

            }
        }
        private void BoutonEnvoyer_Click_1(object sender, RoutedEventArgs e)
        {
            sendMessage();

        }
        private void boutonClear_Click(object sender, RoutedEventArgs e)
        {
            textboxReception.Clear();
        }

        private void boutonTest_Click(object sender, RoutedEventArgs e)
        {
            string testText = "hello world!";
            byte[] testBytes = Encoding.ASCII.GetBytes(testText);
            UartEncodeAndSendMessage(0x0080, testBytes.Length, testBytes);
        }

        private void TextBoxEmission_TextChanged(object sender, TextChangedEventArgs e) { }

        private void TextBoxReception_TextChanged(object sender, TextChangedEventArgs e) { }

        private void CheckBox_Checked(object sender, RoutedEventArgs e) { }
    }

    public class Reception
    {
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

        private StateReception rcvState = StateReception.Waiting;

        private int msgDecodedFunction = 0;
        private int msgDecodedPayloadLength = 0;
        private byte[] msgDecodedPayload;
        private int msgDecodedPayloadIndex = 0;

        public void CallDecodeMessage(byte c)
        {
            DecodeMessage(c);
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
                    msgDecodedFunction |= (c << 8);
                    rcvState = StateReception.FunctionLSB;
                    break;

                case StateReception.FunctionLSB:
                    msgDecodedFunction |= c;
                    rcvState = StateReception.PayloadLengthMSB;
                    break;

                case StateReception.PayloadLengthMSB:
                    msgDecodedPayloadLength |= (c << 8);
                    rcvState = StateReception.PayloadLengthLSB;
                    break;

                case StateReception.PayloadLengthLSB:
                    msgDecodedPayloadLength |= c;
                    if (msgDecodedPayloadLength > 0)
                    {
                        msgDecodedPayload = new byte[msgDecodedPayloadLength];
                        rcvState = StateReception.Payload;
                    }
                    else
                    {
                        rcvState = StateReception.CheckSum;
                    }
                    break;

                case StateReception.Payload:
                    if (msgDecodedPayload != null && msgDecodedPayloadIndex < msgDecodedPayload.Length)
                    {
                        msgDecodedPayload[msgDecodedPayloadIndex++] = c;
                        if (msgDecodedPayloadIndex >= msgDecodedPayloadLength)
                        {
                            rcvState = StateReception.CheckSum;
                        }
                    }
                    else
                    {
                        rcvState = StateReception.Waiting;
                    }
                    break;

                case StateReception.CheckSum:
                    if (CalculateChecksum(msgDecodedFunction, msgDecodedPayloadLength, msgDecodedPayload) == c)
                    {
                        rcvState = StateReception.Waiting;
                    }
                    else
                    {
                        rcvState = StateReception.Waiting;
                    }
                    break;

                default:
                    rcvState = StateReception.Waiting;
                    break;
            }
        }


        private byte CalculateChecksum(int msgFunction, int msgPayloadLength, byte[] msgPayload)
        {
            byte checksum = 0;
            checksum ^= 0xFE;
            checksum ^= (byte)(msgFunction >> 8);
            checksum ^= (byte)(msgFunction);
            checksum ^= (byte)(msgPayloadLength >> 8);
            checksum ^= (byte)(msgPayloadLength);

            foreach (byte b in msgPayload)
            {
                checksum ^= b;
            }

            return checksum;
        }
    }
}
