using DroneSimulationBachelor.Model;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Text.Json;
using System.Text.Json.Serialization;
using System.Threading.Tasks;

namespace DroneSimulationBachelor.Abstractions
{
    [JsonDerivedType(typeof(DataNode), "DataNode")]
    [JsonDerivedType(typeof(CentralServer), "CentralServer")]
    public abstract class WayPoint
    {
        public double X { get; set; }
        public double Y { get; set; }
        public string ID { get; set; }

        [JsonConstructor]
        public WayPoint(double x, double y, string id)
        {
            X = x; Y = y; ID = id;
        }

        public WayPoint(WayPoint toCopy)
        {
            this.X = toCopy.X;
            this.Y = toCopy.Y;
        }

        public WayPoint()
        {

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

        public override string ToString()
        {
            return ID;
        }
    }
}
