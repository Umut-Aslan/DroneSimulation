using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace DroneSimulationBachelor
{
    internal class NearestNeighbourRouteGenerator : IRouteGenerator
    {
        public List<WayPoint> GenerateRoute(List<WayPoint> dataPoints)
        {
            if (dataPoints == null || dataPoints.Count == 0)
                return null;

            List<WayPoint> route = new List<WayPoint>();
            List<WayPoint> unvisited = new List<WayPoint>(dataPoints);
            WayPoint currentPoint = dataPoints[0]; // Start at central server X

            route.Add(currentPoint);
            unvisited.Remove(currentPoint);

            while (unvisited.Count > 0)
            {
                WayPoint nearestNeighbor = FindNearestNeighbor(currentPoint, unvisited);
                route.Add(nearestNeighbor);
                unvisited.Remove(nearestNeighbor);
                currentPoint = nearestNeighbor;
            }

            // Return to central server X
            route.Add(dataPoints[0]);

            return route;
        }

        private WayPoint FindNearestNeighbor(WayPoint point, List<WayPoint> candidates)
        {
            WayPoint? nearestNeighbor = null;
            double shortestDistance = double.MaxValue;

            foreach (WayPoint candidate in candidates)
            {
                double distance = point.DistanceTo(candidate);
                if (distance < shortestDistance)
                {
                    shortestDistance = distance;
                    nearestNeighbor = candidate;
                }
            }
            return nearestNeighbor;
        }
    }
}
