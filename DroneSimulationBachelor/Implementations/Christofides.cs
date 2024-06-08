﻿using DroneSimulationBachelor.Abstractions;
using DroneSimulationBachelor.Model;
using Microsoft.Win32.SafeHandles;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Security.Cryptography.X509Certificates;
using System.Text;
using System.Threading.Tasks;

namespace DroneSimulationBachelor.Implementations
{
    public class Christofides : IRouteGenerator
    {
        public delegate Graph MSTAlgorithm(Graph g);

        MSTAlgorithm mstAlgorithm;

        public Christofides(MSTAlgorithm mstAlg)
        { 
            mstAlgorithm = mstAlg;
        }

        public Christofides()
        {
            mstAlgorithm = MinimumSpanningTree;
        }

        public List<WayPoint> GenerateRoute(List<WayPoint> dataPoints)
        {
            Graph data = new Graph(dataPoints);

            //create Spanning Tree
            Graph T = mstAlgorithm(data);
            return GenerateRoute(data, T);
        }

        public List<WayPoint> GenerateRoute(Graph fullGraph, Graph T)
        {

            //Calculate O = Set of Vertices with odd Degrees in T
            List<WayPoint> O = GetVerticesWithOddDegrees(T);

            //Calculate Subgraph of G using only the vertices of O
            Graph G_Sub = InduceGraph(fullGraph, O);

            //find minimal perfect matching (regarding weight) M with odd degree
            Dictionary<WayPoint, WayPoint> M = FindMinimumWeightPerfectMatching(G_Sub);

            //Combine the edges of M and T to form a connected multigraph H in which each vertex has even degree.
            Graph H = CombineMatchingAndMSTwithEvenDegree(M, T);

            //construct a eulerian tour
            List<WayPoint> eulerianTour = HierholzerEulerianTour(H);

            //Make the circuit found in previous step into a Hamiltonian circuit by skipping repeated vertices (shortcutting).
            List<WayPoint> hamiltonianTour = ConvertEulerianToHamiltonianTour(eulerianTour);

            return hamiltonianTour;
        }

        private Graph InduceGraph(Graph OriginalGraph, List<WayPoint> SetOfVertices)
        {
            Graph inducedGraph = new Graph();

            // Add vertices from the set to the induced graph
            foreach (var vertex in SetOfVertices)
            {
                inducedGraph.Vertices.Add(vertex);
            }

            // Add edges from the original graph that connect vertices in the set
            foreach (var edge in OriginalGraph.Edges)
            {
                if (SetOfVertices.Contains(edge.VertexA) && SetOfVertices.Contains(edge.VertexB))
                {
                    inducedGraph.AddEdge(edge);
                }
            }

            return inducedGraph;
        }

        private List<WayPoint> GetVerticesWithOddDegrees(Graph t)
        {
            List<WayPoint> O = new List<WayPoint>();
            
            foreach(WayPoint vertex in t.Vertices)
            {
                if (t.GetDegree(vertex) % 2 != 0) O.Add(vertex);
            }
            return O;
        }

        public static Graph MinimumSpanningTree(Graph g)
        {
            List<WayPoint> remainingVertices = new(g.Vertices);

            WayPoint currentVertex = remainingVertices.First();
            remainingVertices.Remove(currentVertex);

            // Initialize the MST with the starting vertex
            Graph mst = new Graph();
            mst.Vertices.Add(currentVertex);

            while (remainingVertices.Count > 0)
            {
                Edge minEdge = new();
                minEdge.Distance = Double.PositiveInfinity;
                WayPoint closestVertex = null;
                foreach (WayPoint vertex in mst.Vertices)
                {
                    foreach (WayPoint candidate in remainingVertices)
                    {
                        Edge edge = new(vertex, candidate);
                        if (edge.Distance < minEdge.Distance)
                        {
                            minEdge = edge;
                            closestVertex = candidate;
                        }
                    }
                }

                mst.Edges.Add(minEdge);
                Debug.Assert(closestVertex is not null);
                mst.Vertices.Add(closestVertex);
                bool b = remainingVertices.Remove(closestVertex);

            }
            return mst;
        }

        public static Graph EnforcedEdgeSpanningTree(Graph mst,  Edge edge)
        {
            Graph spanningTree = new Graph(mst);

            if (spanningTree.Edges.Contains(edge)) return spanningTree;
            
            spanningTree.Edges.Add(edge);

            List<Edge> cycle = new() { edge };
            
            bool foundCycle = RecursiveCycleSearch(spanningTree, cycle, edge.VertexA);
            Debug.Assert(foundCycle);

            Edge maxEdge = new();
            maxEdge.Distance = -1;

            foreach(Edge candidate in cycle)
            {
                if (candidate == edge) continue;
                if (candidate.Distance > maxEdge.Distance) maxEdge = candidate;
            }

            spanningTree.RemoveEdge(maxEdge);

            return spanningTree;
        }

        //the first vertex has to be in the cycle, as well as the edge, also the graph has to contain exactly 1 cycle.
        public static bool RecursiveCycleSearch(Graph cycledSpanningTree, List<Edge> visitedEdges, WayPoint lastVertex)
        {
            Edge lastEdge = visitedEdges.Last();
            WayPoint currentVertex = lastEdge.VertexA == lastVertex ? lastEdge.VertexB : lastEdge.VertexA;

            if (visitedEdges.Last() == visitedEdges.First())
            {
                visitedEdges.Remove(visitedEdges.Last());
                return true;
            }

            List<Edge> possibleEdges = cycledSpanningTree.Edges.FindAll(
                (edge) => edge.VertexA == currentVertex || edge.VertexB == currentVertex
            );

            foreach(Edge edge in possibleEdges )
            {
                if (edge == lastEdge) continue;

                visitedEdges.Add(edge);
                if (RecursiveCycleSearch(cycledSpanningTree, visitedEdges, currentVertex))
                    return true;

                visitedEdges.RemoveAt(visitedEdges.Count - 1);
            }
            return false;
        }

        public static Graph ModifiedMST(Graph g)
        {
            List<WayPoint> remainingVertices = new(g.Vertices);

            WayPoint currentVertex = remainingVertices.First();
            remainingVertices.Remove(currentVertex);

            // Initialize the MST with the starting vertex
            Graph mst = new Graph();
            mst.Vertices.Add(currentVertex);
            Edge maxEdge = new();
            maxEdge.Distance = Double.NegativeInfinity;
            WayPoint furthestVertex = null;

            foreach (WayPoint vertex in mst.Vertices)
            {
                foreach(WayPoint candidate in remainingVertices)
                {
                    Edge edge = new(vertex, candidate);
                    if(edge.Distance > maxEdge.Distance)
                    {
                        maxEdge = edge;
                        furthestVertex = candidate;
                    }
                }
            }

            Console.WriteLine($"The longest vertex is {maxEdge.VertexA.DistanceTo(maxEdge.VertexB)}\n");

            mst.Edges.Add(maxEdge);
            mst.Vertices.Add(furthestVertex);
            remainingVertices.Remove(furthestVertex);

            while (remainingVertices.Count > 0)
            {
                Edge minEdge = new();
                minEdge.Distance = Double.PositiveInfinity;
                WayPoint closestVertex = null;
                foreach (WayPoint vertex in mst.Vertices)
                {
                    foreach (WayPoint candidate in remainingVertices)
                    {
                        Edge edge = new(vertex, candidate);
                        if (edge.Distance < minEdge.Distance)
                        {
                            minEdge = edge;
                            closestVertex = candidate;
                        }
                    }
                }

                mst.Edges.Add(minEdge);
                Debug.Assert(closestVertex is not null);
                mst.Vertices.Add(closestVertex);
                bool b = remainingVertices.Remove(closestVertex);

            }
            return mst;
        }
        public Dictionary<WayPoint, WayPoint> FindMinimumWeightPerfectMatching(Graph g)
        {

            var matching = new Dictionary<WayPoint, WayPoint>();
            foreach(var vertex in g.Vertices)
            {
                if (!matching.ContainsKey(vertex))
                {
                    AugmentMatching(vertex, g, matching, new HashSet<WayPoint>());
                }
            }

            var perfectMatching = new Dictionary<WayPoint, WayPoint>();
            
            foreach(var vertex in g.Vertices)
            {
                if (matching.ContainsKey(vertex))
                {
                    var matchedVertex = matching[vertex];
                    perfectMatching[vertex] = matchedVertex;
                    perfectMatching[matchedVertex] = vertex;
                }
            }

            return perfectMatching;
        }

        private bool AugmentMatching(WayPoint vertex, Graph g, Dictionary<WayPoint, WayPoint> matching, HashSet<WayPoint> visited)
        {
            visited.Add(vertex);
            foreach (var edge in g.Edges)
            {
                if (edge.VertexA == vertex)
                {
                    var neighbour = edge.VertexB;
                    if (!matching.ContainsKey(neighbour))
                    {
                        matching[vertex] = neighbour;
                        matching[neighbour] = vertex;
                        return true;
                    }
                    if (visited.Contains(neighbour)) continue;
                    //if (AugmentMatching(matching[neighbour], g, matching, visited))
                    //{
                    //    matching[vertex] = neighbour;
                    //    matching[neighbour] = vertex;
                    //    return true;
                    //}
                }
            }
            return false;
        }
    
        private List<WayPoint> HierholzerEulerianTour(Graph g)
        {
            var eulerianTour = new List<WayPoint>();

            Graph graphClone = new Graph(new List<WayPoint>(g.Vertices));

            foreach(var edge in g.Edges)
            {
                graphClone.AddEdge(new Edge(edge.VertexA, edge.VertexB));
            }

            WayPoint startVertex = null;
            foreach(var vertex in g.Vertices)
            {
                if(graphClone.GetDegree(vertex) % 2 != 0)
                {
                    startVertex = vertex;
                    break;
                }
            }

            if(startVertex != null)
            {
                var stack = new Stack<WayPoint>();
                stack.Push(startVertex);

                while(stack.Count > 0)
                {
                    var currentVertex = stack.Peek();
                    var nextVertex = graphClone.GetAdjacentVertex(currentVertex);

                    if(nextVertex == null)
                    {
                        eulerianTour.Add(stack.Pop());
                    }
                    else
                    {
                        graphClone.RemoveEdge(new Edge(currentVertex, nextVertex));
                        stack.Push(nextVertex);
                    }
                }

                eulerianTour.Reverse();
            }
            return eulerianTour;
        }       

        public List<WayPoint> ConvertEulerianToHamiltonianTour(List<WayPoint> eulerianTour)
        {
            var visited = new HashSet<WayPoint>();
            var hamiltonianTour = new List<WayPoint>();

            foreach(var vertex in eulerianTour)
            {
                if(!visited.Contains(vertex))
                {
                    hamiltonianTour.Add(vertex);
                    visited.Add(vertex);
                }
            }

            foreach(var vertex in eulerianTour)
            {
                if (!hamiltonianTour.Contains(vertex))
                {
                    int index = hamiltonianTour.IndexOf(vertex) + 1;
                    hamiltonianTour.Insert(index, vertex);
                }
            }

            if(hamiltonianTour.Count > 0) hamiltonianTour.Add(hamiltonianTour[0]);
            return hamiltonianTour;
        }

        private Graph CombineMatchingAndMSTwithEvenDegree(Dictionary<WayPoint, WayPoint> matching, Graph MST)
        {
            Graph H = new Graph();

            // Step 1: Create a copy of the minimum spanning tree
            foreach (var edge in MST.Edges)
            {
                H.AddEdge(edge);
                H.Vertices.Add(edge.VertexA); // Add vertices from MST
                H.Vertices.Add(edge.VertexB);
            }

            // Step 2: Add matching edges to the copy of MST
            foreach (var edge in matching)
            {
                WayPoint vertexA = edge.Key;
                WayPoint vertexB = edge.Value;

                H.AddEdge(new Edge(vertexA, vertexB));
                H.Vertices.Add(vertexA); // Add vertices from matching
                H.Vertices.Add(vertexB);
            }

            // Step 3: Track the degree of each vertex in H
            Dictionary<WayPoint, int> vertexDegrees = new Dictionary<WayPoint, int>();
            foreach (var vertex in H.Vertices)
            {
                int degree = H.GetDegree(vertex);
                vertexDegrees[vertex] = degree;
            }

            // Step 4: Ensure each vertex has even degree
            foreach (var vertex in vertexDegrees.Keys.ToList())
            {
                int degree = vertexDegrees[vertex];
                if (degree % 2 != 0)
                {
                    // Add an edge to make the degree even (e.g., to itself)
                    H.AddEdge(new Edge(vertex, vertex));
                }
            }
            return H;
        }

        public override string? ToString()
        {
            return $"christofides_{mstAlgorithm.Method.Name}";
        }
    }
}
