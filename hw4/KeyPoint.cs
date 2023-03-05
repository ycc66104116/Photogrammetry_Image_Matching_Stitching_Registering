using System;
using System.Drawing;

namespace NSURF
{
    [Serializable]
  
    public struct KeyPoint
    {
        public float X, Y;
        public int IntX, IntY;
        public int Scale;
        public double Orientation;  
    }


    public struct Match
    {
        public KeyPoint Kp1;
        public KeyPoint Kp2;

        public Match(KeyPoint Kp1, KeyPoint Kp2)
        {
            this.Kp1 = Kp1;
            this.Kp2 = Kp2;            
        }
    }

}
