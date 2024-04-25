using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace DroneSimulationBachelor
{
    public class MinimumSpanningTreeRouteGenerator : IRouteGenerator
    {
        public List<WayPoint> GenerateRoute(List<WayPoint> dataPoints)
        {
            if (dataPoints == null || dataPoints.Count == 0)
                return null;

            // Create a set to hold vertices not yet included in the MST
            HashSet<WayPoint> remainingVertices = new HashSet<WayPoint>(dataPoints);

            // Choose a starting vertex arbitrarily
            WayPoint startVertex = remainingVertices.First();
            remainingVertices.Remove(startVertex);

            // Initialize the MST with the starting vertex
            List<WayPoint> mst = new List<WayPoint> { startVertex };

            // Initialize a dictionary to store the closest neighbor for each vertex
            Dictionary<WayPoint, WayPoint> closestNeighbors = new Dictionary<WayPoint, WayPoint>();

            // Initialize a dictionary to store the distance to the closest neighbor for each vertex
            Dictionary<WayPoint, double> closestDistances = new Dictionary<WayPoint, double>();

            // Initialize the closest distances for each vertex
            foreach (WayPoint vertex in remainingVertices)
            {
                closestNeighbors[vertex] = startVertex;
                closestDistances[vertex] = startVertex.DistanceTo(vertex);
            }

            // Iterate until all vertices are included in the MST
            while (remainingVertices.Count > 0)
            {
                // Find the vertex with the smallest distance to the MST
                WayPoint nearestVertex = remainingVertices.OrderBy(v => closestDistances[v]).First();

                // Add the nearest vertex to the MST
                mst.Add(nearestVertex);
                remainingVertices.Remove(nearestVertex);

                // Update closest distances and neighbors for the remaining vertices
                foreach (WayPoint vertex in remainingVertices)
                {
                    double distance = nearestVertex.DistanceTo(vertex);
                    if (distance < closestDistances[vertex])
                    {
                        closestDistances[vertex] = distance;
                        closestNeighbors[vertex] = nearestVertex;
                    }
                }
            }

            // Return the MST (route)
            return mst;
        }
    }
}
