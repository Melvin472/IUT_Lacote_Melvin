using System;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Threading;
using ExtendedSerialPort_NS;
using System.IO.Ports;
using WpfOscilloscopeControl;
using System.Windows.Media;
using Utilities;
using SciChart.Charting.Visuals;
using static System.Runtime.InteropServices.JavaScript.JSType;
namespace RobotInterface
{
    public partial class MainWindow : Window
    {

        private ExtendedSerialPort? serialPort1;
        private DispatcherTimer? timerAffichage;
        private Robot robot = new Robot();
        private Reception reception = new Reception();

        private int oscilloLineId = 1;

        public MainWindow()
        {
            SciChartSurface.SetRuntimeLicenseKey("VKOUDZGU6WndydcBQTqx4px2yWsaXqbn+hIKIxA5AE7Vii9ai5FosulEM8j2NYkBkJFZ6Ei2pFlUIV8aoE7bc3FfN3QRUwtvCaGqmrseTOeNsCz9p4t2CBk7TjcTPW7JTOYnIH/UjoRxi8b0BK6MDi8XJUS98gXSybDb/cn070Y5voaiKvusgmvvAOjcwuGcPQuyV7vJlzqh3LqLL3TqJnJMTdGmM00s8VFb7U+sxfbzT/h8SQuY13u/3i5sSz0VEI6YYJeiiX3oMajfHwA/SGyyDFTZmDAAfILtohF7ag+hnEpUDqhudgYjXqVwVtc0oUZNT8Ghtx0ek2bjkQukPtp8/44M1wiOdZORUOCAxeh3oTPZKjEGRjkpbN/UKprgi8/Xvf11BuXzTJLXklmSZLFRsgxcx3nvQVwae9oY5HABtwOk+q/bdsNBKyPmhjNLM1+y5qSlpIQlHzm/EdvN44AX5iR43d4dxfLx9QN7KHvaUbHpqNXVKLUsq0g1g6mEGntw5fXj");
            InitializeComponent();
            InitializeSerialPort();
            InitializeOscilloscope();
            InitializeTimer();
        }

        private void InitializeSerialPort()
        {
            serialPort1 = new ExtendedSerialPort("COM3", 115200, Parity.None, 8, StopBits.One);
            serialPort1.DataReceived += SerialPort1_DataReceived;
            serialPort1.Open();
        }

        private void InitializeOscilloscope()
        {
            oscilloSpeed.AddOrUpdateLine(oscilloLineId, 200, "Vitesse");
            oscilloSpeed.ChangeLineColor(oscilloLineId, Colors.Blue);
        }

        private void InitializeTimer()
        {
            timerAffichage = new DispatcherTimer
            {
                Interval = TimeSpan.FromMilliseconds(100)
            };
            timerAffichage.Tick += TimerAffichage_Tick;
            timerAffichage.Start();
        }

        private void TimerAffichage_Tick(object? sender, EventArgs e)
        {
            while (robot.byteListReceived.Count > 0)
            {
                byte receivedByte = robot.byteListReceived.Dequeue();
                //textBoxReception.Text += receivedByte.ToString("X2") + " ";
                reception.CallDecodeMessage(receivedByte);

                // Exemple : affichage dans oscilloscope
                double x = DateTime.Now.TimeOfDay.TotalSeconds;
                double y = receivedByte; // ou conversion de ta donnée
                oscilloSpeed.AddPointToLine(oscilloLineId, x, y);
            }
        }

        private void SerialPort1_DataReceived(object? sender, DataReceivedArgs e)
        {
            foreach (byte dataByte in e.Data)
            {
                robot.byteListReceived.Enqueue(dataByte);
            }
        }

        private void sendMessage()
        {
            serialPort1?.Write(textBoxEmission.Text);
        }

        private void BoutonEnvoyer_Click_1(object sender, RoutedEventArgs e)
        {
            sendMessage();
        }

        private void boutonClear_Click(object sender, RoutedEventArgs e)
        {
            textBoxReception.Clear();
        }

        private void boutonTest_Click(object sender, RoutedEventArgs e)
        {
            string testText = "hello world!";
            byte[] testBytes = Encoding.ASCII.GetBytes(testText);
            UartEncodeAndSendMessage(0x0080, testBytes.Length, testBytes);
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
            serialPort1?.Write(tram, 0, tram.Length);
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
        void ProcessDecodedMessage(int msgFunction, int msgPayloadLength, byte[] msgPayload)
        {
            switch ((Function)msgFunction)
            {
                case Function.OdometryData:
                    var t = BitConverter.ToSingle(msgPayload, 0);
                    var vitesseLin = BitConverter.ToSingle(msgPayload, 16);

                    break;
            }
        }

        // LED event handlers
        private void led1_Checked(object sender, RoutedEventArgs e) { /* Allumer LED1 */ }
        private void led1_Unchecked(object sender, RoutedEventArgs e) { /* Éteindre LED1 */ }
        private void led2_Checked(object sender, RoutedEventArgs e) { /* Allumer LED2 */ }
        private void led2_Unchecked(object sender, RoutedEventArgs e) { /* Éteindre LED2 */ }
        private void led3_Checked(object sender, RoutedEventArgs e) { /* Allumer LED3 */ }
        private void led3_Unchecked(object sender, RoutedEventArgs e) { /* Éteindre LED3 */ }

        private void TextBoxEmission_TextChanged(object sender, TextChangedEventArgs e) { }
        private void TextBoxReception_TextChanged(object sender, TextChangedEventArgs e) { }
        private void CheckBox_Checked(object sender, RoutedEventArgs e) { }
    }

    enum Function
    {
        OdometryData = 0x0062,
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
        private byte[]? msgDecodedPayload;
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
                    if (c == 0xFE)
                        rcvState = StateReception.FunctionMSB;
                    break;

                case StateReception.FunctionMSB:
                    msgDecodedFunction = (c << 8);
                    rcvState = StateReception.FunctionLSB;
                    break;

                case StateReception.FunctionLSB:
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
                        msgDecodedPayload = new byte[msgDecodedPayloadLength];
                        msgDecodedPayloadIndex = 0;
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
                    if (msgDecodedPayload != null &&
                        CalculateChecksum(msgDecodedFunction, msgDecodedPayloadLength, msgDecodedPayload) == c)
                    {
                        // Données valides
                        ProcessDecodedMessage(msgDecodedFunction, msgDecodedPayloadLength, msgDecodedPayload);
                    }
                    else
                    {
                        // Erreur de checksum
                    }
                    rcvState = StateReception.Waiting;
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


    public class MessageEventArgs : EventArgs
    {
        public int msgFunction { get; set; }
        public int msgPayloadLength { get; set; }
        public byte[] msgPayload { get; set; }
    }
}
