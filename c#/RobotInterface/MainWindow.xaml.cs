using System.Security.Cryptography.X509Certificates;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using ExtendedSerialPort_NS;
using System.IO.Ports;
using System.Diagnostics.Tracing;
using System.Runtime.Serialization;
using System.Windows.Threading;
using static SciChart.Drawing.Utility.PointUtil;
using SciChart.Data.Model;
using SciChart.Charting.Visuals;

using System.Windows.Media;
using WpfOscilloscopeControl;

namespace RobotInterface
{

    public partial class MainWindow : Window
    {
        bool rawTramDspMode = true;
        bool posDspMode = false;
        bool speedDspMode = false;
        bool distDspMode = false;

        ExtendedSerialPort serialPort1;
        DispatcherTimer timerAffichage;
        
        Robot Robot = new Robot();
        String receptionWindowDspMode = "txt";
        public MainWindow()
        {
            // Set this code once in App.xaml.cs or application startup
            SciChartSurface.SetRuntimeLicenseKey("rawGYdZucXOANEX0TnQ0wSvPHXM3trkTCdRqc9VgfXhPE3bF05t7I6j41xW49IvBviM4ep3kbiO/Gj9qQ3rNDcq89Lm8hXfqu/0rMuj5hoTrcn1knJIGGB85+GaoUQP4ZV+mMFPnL3b2erT/NXobKdwi9SmlNJHfoesS4uM7AvLOSUymT8FknehVb1Ur9rqr1jxMn37sDcBql2nBHuDDSXiX1Fl0EB7OiMKyPZf+bEdqKE+j4+oOyZIGvuXbIAnffl4b1jv5J9vW2LmlDptXKLPQ42oywJwL+GmejhqV6VMSWB/9wtPfHKrPxRAI+7ViMMoKyIwR0Fch+PoLLRvA2ACLjXza3JCp3+SGXEGnajE8g+Wyey6gqIbxz1dGhEq8QXQa3X/QPfrZb4G/B+SFgo/Q8FJGn4sxCV+n469Wr92YF+d23giEOt49UFn2IL6czW19nFUQF09pRX6buAq0ULeFFeQs0vB0uxH+aosgIr5SuR6+W+9ptFSxJf+XZiA44wbhney7");

            InitializeComponent();
            serialPort1 = new ExtendedSerialPort("COM11", 115200, Parity.None, 8, StopBits.One);
            serialPort1.DataReceived += SerialPort1_DataReceived;
            serialPort1.Open();
            timerAffichage = new DispatcherTimer();
            timerAffichage.Interval = new TimeSpan(0, 0, 0, 0, 1);
            timerAffichage.Tick += TimerAffichage_Tick1;
            timerAffichage.Start();



            oscilloDirection.isDisplayActivated = true;
            oscilloSpeed.isDisplayActivated = true;
            oscilloDistance.isDisplayActivated = true;
            oscilloDirection.AddOrUpdateLine(0, 200, "Direct. & Speed");
            oscilloSpeed.AddOrUpdateLine(0, 200, "Speed lin.");
            oscilloDistance.AddOrUpdateLine(0, 200, "Distance");




        }

        public void SerialPort1_DataReceived(object sender, DataReceivedArgs e)
        {
            for (int i = 0; i < e.Data.Length; i++)
            {
                Robot.byteListReceived.Enqueue(e.Data[i]);
            }
        }

        private void TimerAffichage_Tick1(object? sender, EventArgs e)
        {
            byte[] temp = new byte[64];
            
            for (int i = 0; i < Robot.byteListReceived.Count; i++) {
                
                byte receivedByte = Robot.byteListReceived.Dequeue();
                receptionWindowDisplay(receivedByte.ToString("X2"), "txt"); //affichage de tram
                Robot.CallDecodeMessage(receivedByte ); //traitement de tram
            }
            
            oscilloDirection.AddPointToLine(0, Double.Parse((Robot.angleRadianFromOdometry* 57.2958).ToString()), Double.Parse((Robot.vitesseAngulaireFromOdometry).ToString()));
            oscilloSpeed.AddPointToLine(0, Double.Parse((Robot.timestamp).ToString()), Double.Parse((Robot.vitesseLineaireFromOdometry).ToString()));
            oscilloDistance.AddPointToLine(0, Double.Parse(0.ToString()), Double.Parse((Robot.sensor[2]/ 100.0).ToString()));
            Console.WriteLine((Robot.distanceTelemetreCentre / 100.0).ToString());
            oscilloDistance.AddPointToLine(0, Double.Parse(30.ToString()), Double.Parse(((Robot.sensor[4] / 100.0).ToString() ).ToString()));

        }
        public void receptionWindowDisplay(string textReceived, string dataToDspType)
        {
            if (textReceived == "FE")
            {
                TextBoxReception.Text = ""; // On vide à chaque nouvelle tramme -> TODO faire un buffer circulaire 
                if (distDspMode)
                {
                    if (Robot.sensor[0] != 0)
                    {
                        TextBoxReception.Text += "------------Distance\n";
                        TextBoxReception.Text += "Distance G: " + Robot.sensor[0].ToString() + "\n";
                        TextBoxReception.Text += "Distance C: " + Robot.sensor[2].ToString() + "\n";
                        TextBoxReception.Text += "Distance D: " + Robot.sensor[4].ToString() + "\n\n";
                    }
                    else
                    {
                        TextBoxReception.Text += "------------Distance\n";
                        TextBoxReception.Text += "No data from robot" + "\n\n";
                    }
                }
                if (posDspMode)
                {
                    TextBoxReception.Text += "------------Position\n";
                    TextBoxReception.Text += "Timestamp: " + (Robot.timestamp).ToString() + "\n";
                    TextBoxReception.Text += "xPosFromOdometry: " + (Robot.xPosFromOdometry).ToString() + "\n";
                    TextBoxReception.Text += "yPosFromOdometry: " + (Robot.yPosFromOdometry).ToString() + "\n";
                    TextBoxReception.Text += "angleRadianFromOdometry: " + (Robot.angleRadianFromOdometry).ToString() + "\n";
                    TextBoxReception.Text += "vitesseLineaireFromOdometry: " + (Robot.vitesseLineaireFromOdometry).ToString() + "\n";
                    TextBoxReception.Text += "vitesseAngulaireFromOdometry: " + (Robot.vitesseAngulaireFromOdometry).ToString() + "\n\n";
                }
                if (rawTramDspMode)
                {
                    TextBoxReception.Text += "------------Raw data\n";
                }
                }
            if (rawTramDspMode) {
                
                TextBoxReception.Text += textReceived; }
        }
           
            

        private void TextBoxEmission_TextChanged()
        {

        }

        private void TextBoxEmission_TextChanged(object sender, TextChangedEventArgs e)
        {
        }

        void UartEncodeAndSendMessage(int msgFunction, int msgPayloadLength, byte[] msgPayload) {
            byte[] tram = new byte[6 + msgPayloadLength];
            tram[0] = 0xFE;
            tram[1] = (byte)(msgFunction >> 8);
            tram[2] = (byte)(msgFunction);
            tram[3] = (byte)(msgPayloadLength >> 8);
            tram[4] = (byte)(msgPayloadLength);

            for (int i = 5; i < msgPayload.Length + 5; i++)
            {
               
                tram[i] = (byte)(msgPayload[i - 5]);
            }
            byte checkSum = CalculateChecksum(msgFunction, msgPayloadLength, msgPayload);
            tram[5 + msgPayloadLength] = checkSum;
            serialPort1.Write(tram, 0, 6 + msgPayloadLength);

        }
        //bool a = false;
        byte CalculateChecksum(int msgFunction, int msgPayloadLength, byte[] msgPayload)
        {
            byte checksum = 0;
            checksum ^= 0xFE;
            checksum ^= (byte)(msgFunction >> 8);
            checksum ^= (byte)(msgFunction >> 0);
            checksum ^= (byte)(msgPayloadLength >> 8);
            checksum ^= (byte)(msgPayloadLength >> 0);


            for (int i = 0; i < msgPayload.Length - 1; i++) 
            {
                checksum ^= msgPayload[i];
            }
            return checksum;

        }

        private void BoutonEnvoyer_Click_1(object sender, RoutedEventArgs e)
        {
            SendMessage();
        }

        private void TextBoxEmission_KeyUp_1(object sender, KeyEventArgs e)
        {
            if (e.Key == Key.Enter)
            {
                SendMessage();
            }
        }

        private void SendMessage()
        {
            /*bool textFormat = (bool)textCheckBox.IsChecked;
            bool LEDFormat = (bool)ledCheckBox.IsChecked;
            bool speedtFormat = (bool)speedCheckBox.IsChecked;
            bool distFormat = (bool)distCheckBox.IsChecked;

            byte[] bytesToSend = Encoding.ASCII.GetBytes(TextBoxEmission.Text);
            


            if (textFormat && LEDFormat && speedtFormat && distFormat)
            { }
            else if (textFormat)
            {
                
                UartEncodeAndSendMessage(0x0080, bytesToSend.Length, bytesToSend);
            }
            else if (LEDFormat)
            {
                UartEncodeAndSendMessage(0x0020, bytesToSend.Length, bytesToSend);
            }
            else if (speedtFormat)
            {
                String textToSend = TextBoxEmission.Text;
                String motor = textToSend;
                UartEncodeAndSendMessage(0x0040, bytesToSend.Length, bytesToSend);
            }
            else if (distFormat)
            {
                UartEncodeAndSendMessage(0x0030, bytesToSend.Length, bytesToSend);
            }
            else { TextBoxReception.Text += "At least one format has to be choosen\n"; }
            TextBoxEmission.Text = "";
            */


        }

      

        private void boutonClear_Click(object sender, RoutedEventArgs e)
        {
            TextBoxReception.Text = ""; 
        }

        private void boutonTest_Click(object sender, RoutedEventArgs e)
        {
            string text = "Bonjour";
            byte[] temp = Encoding.ASCII.GetBytes(text);
            UartEncodeAndSendMessage(0x0080, temp.Length, temp);
        }
        
        private void TextBox_TextChanged(object sender, TextChangedEventArgs e)
        {

        }

        private void TextBoxReception_TextChanged(object sender, TextChangedEventArgs e)
        {

        }

        private void DisplayModeCheckBox_Checked(object sender, RoutedEventArgs e)
        {
            if (sender is CheckBox checkBox)
            {
                switch (checkBox.Name)
                {
                    case "textCheckBox":
                        rawTramDspMode = true;
                        break;
                    case "distCheckBox":
                        distDspMode = true;
                        break;
                    case "posCheckBox":
                        posDspMode = true;
                        break;
                }
            }
        }
        private void DisplayModeCheckBox_Unchecked(object sender, RoutedEventArgs e)
        {
            if (sender is CheckBox checkBox)
            {
                switch (checkBox.Name)
                {
                    case "textCheckBox":
                        rawTramDspMode = false;
                        break;
                    case "distCheckBox":
                        distDspMode = false;
                        break;
                    case "posCheckBox":
                        posDspMode = false;
                        break;
                }
            }
        }

        private void TextBox_TextChanged_1(object sender, TextChangedEventArgs e)
        {

        }

       
    }
}