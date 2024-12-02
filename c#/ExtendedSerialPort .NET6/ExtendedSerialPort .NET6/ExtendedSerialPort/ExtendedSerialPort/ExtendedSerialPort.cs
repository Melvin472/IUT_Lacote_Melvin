using System;
using System.IO.Ports;
using System.Management;
using System.Text.RegularExpressions;
using System.Threading;

namespace ExtendedSerialPort_NS
{
    public class ExtendedSerialPort : SerialPort
    {
        private Thread connectionThread;
        private bool IsSerialPortConnected = false;
        private readonly ManualResetEvent isThreadActive = new(false);

        public ExtendedSerialPort(string portName, int baudRate, Parity parity, int dataBits, StopBits stopBits)
        {
            PortName = portName;
            BaudRate = baudRate;
            DataBits = dataBits;
            Parity = parity;
            StopBits = stopBits;
            Handshake = Handshake.None;
            DtrEnable = true;
            NewLine = Environment.NewLine;
            ReceivedBytesThreshold = 1024;

            connectionThread = new Thread(ConnectionThreadMethod)
            {
                IsBackground = true
            };
            connectionThread.Start();
            StartTryingToConnect();
        }

        private void ConnectionThreadMethod()
        {
            while (true)
            {
                if (isThreadActive.WaitOne())
                {
                    string portNameFound = PortName; 
                    if (!string.IsNullOrWhiteSpace(portNameFound))
                    {
                        base.PortName = portNameFound;
                        try
                        {
                            base.Open();
                            IsSerialPortConnected = true;
                            Console.WriteLine("Connection to serial port successful.");
                            ContinuousRead();
                            StopTryingToConnect();
                        }
                        catch
                        {
                            IsSerialPortConnected = false;
                            Console.WriteLine("Connection to serial port failed.");
                        }
                    }
                    else
                    {
                        IsSerialPortConnected = false;
                        Console.WriteLine("Serial port not found.");
                    }
                    Thread.Sleep(2000);
                }
            }
        }

        private void StartTryingToConnect()
        {
            isThreadActive.Set();
        }

        private void StopTryingToConnect()
        {
            isThreadActive.Reset();
        }

        public new void Open()
        {
          
        }

        private string SearchPortName(string vendorName)
        {
            try
            {

#if WINDOWS
                ManagementObjectSearcher searcher = new("root\\CIMV2", "SELECT * FROM Win32_PnPEntity");
                foreach (ManagementObject queryObj in searcher.Get())
                {
                    // Vérifiez la nullité avant d'accéder aux propriétés
                    string? deviceId = queryObj["DeviceID"]?.ToString();
                    if (deviceId != null && deviceId.Contains(vendorName))
                    {
                        string? caption = queryObj["Caption"]?.ToString();
                        if (caption != null)
                        {
                            Match match = Regex.Match(caption, @"\((COM[0-9]+?)\)", RegexOptions.IgnoreCase);
                            if (match.Success)
                                return match.Groups[1].Value; // Utilisez .Value au lieu de ToString()
                        }
                    }
                }
#endif
                return string.Empty;
            }
            catch
            {
                return string.Empty;
            }
        }

        private void ContinuousRead()
        {
            byte[] buffer = new byte[4096];

            
            void KickoffRead()
            {
                BaseStream?.BeginRead(buffer, 0, buffer.Length, ar =>
                {
                    try
                    {
                        int count = BaseStream?.EndRead(ar) ?? 0;
                        if (count > 0)
                        {
                            byte[] dst = new byte[count];
                            Buffer.BlockCopy(buffer, 0, dst, 0, count);
                            OnDataReceived(dst);
                        }
                    }
                    catch
                    {
                        IsSerialPortConnected = false;
                    }

                    if (IsSerialPortConnected)
                    {
                        KickoffRead(); 
                    }
                }, null);
            }

            KickoffRead(); 
        }

        public void SendMessage(object sender, byte[] msg)
        {
            if (IsSerialPortConnected)
            {
                try
                {
                    Write(msg, 0, msg.Length);
                }
                catch
                {
                    IsSerialPortConnected = false;
                    StartTryingToConnect();
                }
            }
        }

        public new event EventHandler<DataReceivedArgs>? DataReceived;

        protected virtual void OnDataReceived(byte[] data)
        {
            DataReceived?.Invoke(this, new DataReceivedArgs { Data = data });
        }
    }

    public class DataReceivedArgs : EventArgs
    {
        public byte[]? Data { get; set; }
    }
}
