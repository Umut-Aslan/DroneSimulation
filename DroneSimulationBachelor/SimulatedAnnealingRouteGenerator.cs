using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace DroneSimulationBachelor
{
    public class SimulatedAnnealingRouteGenerator : IRouteGenerator
    {
        private readonly double InitialTemperature = 1000;
        private readonly double CoolingRate = 0.003;

        public List<WayPoint> GenerateRoute(List<WayPoint> dataPoints)
        {
            if (dataPoints == null || dataPoints.Count == 0)
                return null;

            List<WayPoint> currentRoute = new List<WayPoint>(dataPoints);
            List<WayPoint> bestRoute = new List<WayPoint>(dataPoints);

            double currentEnergy = CalculateTotalDistance(currentRoute);
            double bestEnergy = currentEnergy;

            double temperature = InitialTemperature;

            while (temperature > 1)
            {
                List<WayPoint> newRoute = GenerateNeighborRoute(currentRoute);
                double newEnergy = CalculateTotalDistance(newRoute);

                if (AcceptanceProbability(currentEnergy, newEnergy, temperature) > RandomDouble())
                {
                    currentRoute = newRoute;
                    currentEnergy = newEnergy;
                }

                if (currentEnergy < bestEnergy)
                {
                    bestRoute = new List<WayPoint>(currentRoute);
                    bestEnergy = currentEnergy;
                }

                temperature *= 1 - CoolingRate;
            }

            return bestRoute;
        }

        private List<WayPoint> GenerateNeighborRoute(List<WayPoint> route)
        {
            List<WayPoint> newRoute = new List<WayPoint>(route);

            int index1 = RandomInteger(0, route.Count - 1);
            int index2 = RandomInteger(0, route.Count - 1);

            WayPoint temp = newRoute[index1];
            newRoute[index1] = newRoute[index2];
            newRoute[index2] = temp;

            return newRoute;
        }

        private double CalculateTotalDistance(List<WayPoint> route)
        {
            double totalDistance = 0;
            for (int i = 0; i < route.Count - 1; i++)
            {
                totalDistance += route[i].DistanceTo(route[i + 1]);
            }
            // Return to the starting point (central server X)
            totalDistance += route.Last().DistanceTo(route.First());
            return totalDistance;
        }

        private double AcceptanceProbability(double energy, double newEnergy, double temperature)
        {
            if (newEnergy < energy)
            {
                return 1.0;
            }
            return Math.Exp((energy - newEnergy) / temperature);
        }

        private static Random random = new Random();

        private static double RandomDouble()
        {
            return random.NextDouble();
        }

        private static int RandomInteger(int min, int max)
        {
            return random.Next(min, max + 1);
        }
    }
}
