using System.Text;
using System.Windows;
using ExtendedSerialPort_NS;
using System.Windows.Controls;
using System.Windows.Controls.Primitives;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;

namespace RobotInterface
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        ExtendedSerialPort serialPort1;

        public MainWindow()
        {
            InitializeComponent();
        }
        bool toggle = false;
        private void SendMessage()
        {
            string value = textboxEmission.Text;            
            textboxReception.Text += "Recu : " + value + "\n";
            textboxEmission.Text = "";
        }

        private void buttonEnvoyer_Click_1(object sender, RoutedEventArgs e)
        {
            if (toggle == true)
            {
                buttonEnvoyer.Background = Brushes.RoyalBlue;
                toggle = false;
            }
            else
            {
                buttonEnvoyer.Background = Brushes.Beige;
                toggle = true;
            }
            textboxReception.Text += textboxEmission.Text + "\n";
            

            textboxEmission.Text = "";

        }

        private void textboxEmission_KeyUp_1(object sender, KeyEventArgs e)
        {
            if (e.Key == Key.Enter)
            {
                SendMessage();
            }

        }
    }
}