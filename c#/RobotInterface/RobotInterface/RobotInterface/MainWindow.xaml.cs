using System;
using System.Text;
using System.Windows;
using ExtendedSerialPort_NS;
using System.Windows.Input;
using System.IO.Ports;

namespace RobotInterface
{
    public partial class MainWindow : Window
    {
        ExtendedSerialPort serialPort1;
        private Robot robot;

        public MainWindow()
        {
            robot = new Robot();
            serialPort1 = new ExtendedSerialPort("COM4", 115200, Parity.None, 8, StopBits.One);
            serialPort1.DataReceived += SerialPort1_DataReceived; // Souscription à l'événement
            serialPort1.Open();
            InitializeComponent();
        }

        private void SendMessage()
        {
            string value = textboxEmission.Text;
            if (!string.IsNullOrEmpty(value))
            {
                robot.receivedText += value + "\n";
                serialPort1.WriteLine(value);
                textboxEmission.Clear();
            }
        }

        public void SerialPort1_DataReceived(object? sender, DataReceivedArgs e)
        {
            if (e.Data != null) // Vérification pour éviter les erreurs de nullabilité
            {
                foreach (byte b in e.Data)
                {
                    robot.byteListReceived.Enqueue(b);
                }

                foreach (byte b in e.Data)
                {
                    serialPort1.Write(new byte[] { b }, 0, 1);
                }
            }
        }

        private void TimerAffichage_Tick(object sender, EventArgs e)
        {
            if (robot.byteListReceived.Count > 0)
            {
                StringBuilder hexString = new StringBuilder();
                while (robot.byteListReceived.Count > 0)
                {
                    byte b = robot.byteListReceived.Dequeue();
                    hexString.Append(b.ToString("X2") + " ");
                }
                textboxReception.Text += "\n" + hexString.ToString().Trim();
            }
        }

        private void buttonEnvoyer_Click_1(object sender, RoutedEventArgs e)
        {
            SendMessage();
        }

        private void textboxEmission_KeyUp_1(object sender, KeyEventArgs e)
        {
            if (e.Key == Key.Enter)
            {
                SendMessage();
            }
        }

        private void buttonClear_Click(object sender, RoutedEventArgs e)
        {
            textboxReception.Clear();
            robot.receivedText = "";
        }

        private void buttonTest_Click(object sender, RoutedEventArgs e)
        {
            byte[] byteList = new byte[20];
            for (int i = 0; i < byteList.Length; i++)
            {
                byteList[i] = (byte)(2 * i);
            }

            serialPort1.Write(byteList, 0, byteList.Length);
        }
    }
}
