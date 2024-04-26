using DroneSimulationBachelor.Abstractions;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace DroneSimulationBachelor.Implementations
{
    public class Christofides : IRouteGenerator
    {
        public List<WayPoint> GenerateRoute(List<WayPoint> dataPoints)
        {
            //create a MST T for Graph G(V,E)
            Graph F = MinimumSpanningTree(new Graph(dataPoints));
            //find minimal perfect matching (regarding weight) M with odd degree
            Graph M = MinimumCostPerfectMatchingOddDegree(F);
            //add Edges to T

            //construct a eulerian tour

            //construct hamiltonian tour.
        }

        public Graph MinimumSpanningTree(Graph g)
        {
            // Create a set to hold vertices not yet included in the MST "01"
            HashSet<WayPoint> remainingVertices = new HashSet<WayPoint>(g.AdjacencyList.Keys);

            WayPoint currentVertex = remainingVertices.First();
            remainingVertices.Remove(currentVertex);

            // Initialize the MST with the starting vertex
            Graph mst = new Graph();
            mst.AddVertex(currentVertex);

            // Initialize a dictionary to store the closest neighbor for each vertex
            Dictionary<WayPoint, WayPoint> closestNeighbors = new Dictionary<WayPoint, WayPoint>();

            // Initialize a dictionary to store the distance to the closest neighbor for each vertex
            Dictionary<WayPoint, double> closestDistances = new Dictionary<WayPoint, double>();

            // Initialize the closest distances for each vertex
            foreach (WayPoint vertex in remainingVertices)
            {
                closestNeighbors[vertex] = currentVertex;
                closestDistances[vertex] = currentVertex.DistanceTo(vertex);
            }

            // Iterate until all vertices are included in the MST
            while (remainingVertices.Count > 0)
            {
                // Find the vertex with the smallest distance to the MST
                WayPoint nearestVertex = remainingVertices.OrderBy(v => closestDistances[v]).First();

                // Add the nearest vertex to the MST
                mst.AddVertex(nearestVertex);
                mst.AddEdge(currentVertex, nearestVertex, currentVertex.DistanceTo(nearestVertex));
                WayPoint temp = nearestVertex;
                currentVertex = temp;
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
        public List<WayPoint> MinimumCostPerfectMatchingOddDegree(Graph g)
        {

        }
    }
}
