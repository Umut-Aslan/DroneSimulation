using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace DroneSimulationBachelor
{
    public abstract class WayPoint
    {
        public double X { get; set; }
        public double Y { get; set; }

        public WayPoint(double x, double y)
        {
            X = x; Y = y;
        }

        public double DistanceTo(WayPoint other)
        {
            double deltaX = X - other.X;
            double deltaY = Y - other.Y;
            return Math.Sqrt(deltaX * deltaX + deltaY * deltaY);
        }
    }
}
