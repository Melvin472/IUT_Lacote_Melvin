using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Collections.Generic;
using Utilities;

namespace RobotInterface
{

    public class Robot
    {
        public Queue<byte> byteListReceived = new Queue<byte>();
        public string receivedText = "";

        public float distanceTelemetreDroit;
        public float distanceTelemetreCentre;
        public float distanceTelemetreGauche;
        public float PositionXOdo { get; set; }
        public float PositionYOdo { get; set; }
        public float AngleRadian { get; set; }
        public float VitesseLineaire { get; set; }
        public float VitesseAngulaire { get; set; }
        public Robot()
        {
        }
    }


}

