using System;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Threading;
using ExtendedSerialPort_NS;
using System.IO.Ports;
using WpfOscilloscopeControl;
using System.Windows.Media;
using SciChart.Charting.Visuals;

namespace RobotInterface
{
    public partial class MainWindow : Window
    {
        private ExtendedSerialPort? serialPort1;
        private DispatcherTimer? timerAffichage;
        private Robot robot = new Robot();   // ta classe Robot déjà définie ailleurs
        private Reception reception = new Reception();

        private int oscilloLineId = 1;

        // ⚡ Pilotage auto/manuel
        private bool autoControlActivated = true;

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
                reception.CallDecodeMessage(receivedByte, ProcessDecodedMessage);

                double x = DateTime.Now.TimeOfDay.TotalSeconds;
                double y = receivedByte;
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
            UartEncodeAndSendMessage((int)Function.TransmissionTexte, testBytes.Length, testBytes);
        }

        // ⚡ Nouvelle fonction : forcer état robot
        private void SendSetRobotState(StateRobot newState)
        {
            byte[] payload = new byte[1];
            payload[0] = (byte)newState;
            UartEncodeAndSendMessage((int)Function.SetRobotState, payload.Length, payload);
            textBoxReception.Text += $"\n[PC] Envoi SetRobotState -> {newState}";
        }

        // ⚡ Nouvelle fonction : activer/désactiver mode auto
        private void SendSetRobotAutoControl(bool activateAuto)
        {
            byte[] payload = new byte[1];
            payload[0] = (byte)(activateAuto ? 1 : 0);
            UartEncodeAndSendMessage((int)Function.SetRobotManualControl, payload.Length, payload);

            autoControlActivated = activateAuto;
            textBoxReception.Text += $"\n[PC] Mode autoControlActivated = {autoControlActivated}";
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
                case Function.TransmissionTexte:
                    string texte = Encoding.ASCII.GetString(msgPayload);
                    textBoxReception.Text += "\nReçu : " + texte;
                    break;

                case Function.ReglageLED:
                    int ledNum = msgPayload[0];
                    bool etat = msgPayload[1] == 1;
                    switch (ledNum)
                    {
                        case 1: led1.IsChecked = etat; break;
                        case 2: led2.IsChecked = etat; break;
                        case 3: led3.IsChecked = etat; break;
                    }
                    textBoxReception.Text += $"\nLED {ledNum} -> {(etat ? "Allumée" : "Éteinte")}";
                    break;

                case Function.DistancesTelemetre:
                    if (msgPayloadLength >= 3)
                    {
                        int dG = msgPayload[0];
                        int dC = msgPayload[1];
                        int dD = msgPayload[2];
                        textboxValeursTelemetres.Text =
                            $"Télémètre Centre : {dC} cm\n" +
                            $"Télémètre Gauche : {dG} cm\n" +
                            $"Télémètre Droit : {dD} cm";
                    }
                    break;

                case Function.ConsigneVitesse:
                    if (msgPayloadLength >= 2)
                    {
                        int vG = msgPayload[0];
                        int vD = msgPayload[1];
                        textboxValeursMoteur.Text =
                            $"Moteur Gauche : {vG}%\nMoteur Droit : {vD}%";
                    }
                    break;

                case Function.RobotState:
                    if (msgPayloadLength >= 5)
                    {
                        int etape = msgPayload[0];
                        int instant = (msgPayload[1] << 24) + (msgPayload[2] << 16) +
                                      (msgPayload[3] << 8) + msgPayload[4];
                        textBoxReception.Text +=
                            $"\nRobot State : {(StateRobot)etape} - {instant} ms";
                    }
                    break;

                case Function.OdometryData:
                    float theta = BitConverter.ToSingle(msgPayload, 0);
                    float vitesseLin = BitConverter.ToSingle(msgPayload, 16);
                    textBoxViutessLineaire.Text = $"{vitesseLin:F2} m/s";
                    textBoxViutessAngulaire.Text = $"{theta:F2} rad/s";
                    textBlockVitesseActuelle.Text = $"{vitesseLin:F2} m/s";
                    break;

                default:
                    textBoxReception.Text += $"\nMessage inconnu : {msgFunction:X4}";
                    break;
            }
        }

        // Gestion LEDs
        private void led1_Checked(object sender, RoutedEventArgs e) { }
        private void led1_Unchecked(object sender, RoutedEventArgs e) { }
        private void led2_Checked(object sender, RoutedEventArgs e) { }
        private void led2_Unchecked(object sender, RoutedEventArgs e) { }
        private void led3_Checked(object sender, RoutedEventArgs e) { }
        private void led3_Unchecked(object sender, RoutedEventArgs e) { }

        private void TextBoxEmission_TextChanged(object sender, TextChangedEventArgs e) { }
        private void TextBoxReception_TextChanged(object sender, TextChangedEventArgs e) { }
        private void CheckBox_Checked(object sender, RoutedEventArgs e) { }
    }

    enum Function
    {
        TransmissionTexte = 0x0080,
        ReglageLED = 0x0020,
        DistancesTelemetre = 0x0030,
        ConsigneVitesse = 0x0040,
        RobotState = 0x0050,
        OdometryData = 0x0062,
        SetRobotState = 0x0051,
        SetRobotManualControl = 0x0052
    }

    public enum StateRobot
    {
        STATE_ATTENTE = 0,
        STATE_ATTENTE_EN_COURS = 1,
        STATE_AVANCE = 2,
        STATE_AVANCE_EN_COURS = 3,
        STATE_TOURNE_GAUCHE = 4,
        STATE_TOURNE_GAUCHE_EN_COURS = 5,
        STATE_TOURNE_DROITE = 6,
        STATE_TOURNE_DROITE_EN_COURS = 7,
        STATE_TOURNE_SUR_PLACE_GAUCHE = 8,
        STATE_TOURNE_SUR_PLACE_GAUCHE_EN_COURS = 9,
        STATE_TOURNE_SUR_PLACE_DROITE = 10,
        STATE_TOURNE_SUR_PLACE_DROITE_EN_COURS = 11,
        STATE_ARRET = 12,
        STATE_ARRET_EN_COURS = 13,
        STATE_RECULE = 14,
        STATE_RECULE_EN_COURS = 15
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

        public void CallDecodeMessage(byte c, Action<int, int, byte[]> processCallback)
        {
            DecodeMessage(c, processCallback);
        }

        private void DecodeMessage(byte c, Action<int, int, byte[]> processCallback)
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
                        processCallback(msgDecodedFunction, msgDecodedPayloadLength, msgDecodedPayload);
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
}
