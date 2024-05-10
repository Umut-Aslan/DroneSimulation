using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace DroneSimulationBachelor.Abstractions
{
    public class WayPoint
    {
        public double X { get; set; }
        public double Y { get; set; }

        public WayPoint(double x, double y)
        {
            X = x; Y = y;
        }

        public WayPoint()
        {
            
        }

        public WayPoint(WayPoint toCopy)
        {
            this.X = toCopy.X;
            this.Y = toCopy.Y;
        }

        public double DistanceTo(WayPoint other)
        {
            double deltaX = X - other.X;
            double deltaY = Y - other.Y;
            return Math.Sqrt(deltaX * deltaX + deltaY * deltaY);
        }

        public override bool Equals(object? obj)
        {
            return obj is WayPoint point &&
                   X == point.X &&
                   Y == point.Y;
        }

        public static bool operator ==(WayPoint? left, WayPoint? right)
        {
            return EqualityComparer<WayPoint>.Default.Equals(left, right);
        }

        public static bool operator !=(WayPoint? left, WayPoint? right)
        {
            return !(left == right);
        }
    }
}
