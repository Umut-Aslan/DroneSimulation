using DroneSimulationBachelor.Abstractions;
using DroneSimulationBachelor.Model;
using Microsoft.Win32.SafeHandles;
using ScottPlot.Control;
using ScottPlot.MarkerShapes;
using ScottPlot.Palettes;
using System;
using System.Collections.Generic;
using System.ComponentModel.Design;
using System.Diagnostics;
using System.Diagnostics.Contracts;
using System.Linq;
using System.Security.Cryptography;
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

        public List<WayPoint> GenerateRoute(Graph fullGraph, Graph T, Edge enforcedEdge = null)
        {

            //Calculate O = Set of Vertices with odd Degrees in T
            List<WayPoint> O = GetVerticesWithOddDegrees(T);

            //Calculate Subgraph of G using only the vertices of O
            Graph G_Sub = InduceGraph(fullGraph, O);

            //find minimal perfect matching (regarding weight) M with odd degree
            Dictionary<WayPoint, WayPoint> M = FindMinimumWeightPerfectMatching(fullGraph, G_Sub);

            //Combine the edges of M and T to form a connected multigraph H in which each vertex has even degree.
            Graph H = CombineMatchingAndSpanningTree(M, T);

            //construct a eulerian tour
            List<WayPoint> eulerianTour = HierholzerEulerianTour(H);

            eulerianTour = ChangeStartingPoint(eulerianTour, enforcedEdge);

            //Make the circuit found in previous step into a Hamiltonian circuit by skipping repeated vertices (shortcutting).
            List<WayPoint> hamiltonianTour = ConvertEulerianToHamiltonianTour(eulerianTour);

            return hamiltonianTour;
        }

        private List<WayPoint> ChangeStartingPoint(List<WayPoint> eulerianTour, Edge enforcedEdge)
        {
            if (enforcedEdge == null) return eulerianTour;
            int length = eulerianTour.Count-1;

            for(int i = 0; i < length; i++)
            {
                Edge currentEdge = new Edge(eulerianTour[i], eulerianTour[(i+1)]);
                if (currentEdge == enforcedEdge)
                {
                    if(eulerianTour[i] is CentralServer)
                    {
                        eulerianTour.RemoveRange(0, i);
                        return eulerianTour;
                    }
                    else if (eulerianTour[i+1] is CentralServer)
                    {
                        eulerianTour.Add(eulerianTour[i + 1]);
                        eulerianTour.RemoveRange(0, i+1);
                        eulerianTour.Reverse();
                        return eulerianTour;
                    }
                    else
                    {
                        Debug.Assert(false);
                    }
                }
                eulerianTour.Add(eulerianTour[i + 1]);
            }
            Debug.Assert(false);
            return eulerianTour;
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

            //Falls die Kante vorhanden aber nicht als erstes drin ist, so muss diese Kante zuerst vorkommen und anschließend der restliche Spannbaum aufgebaut werden.
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

            Edge firstEdge = visitedEdges.First();
            bool firstEdgesContainsCurrentVertex = currentVertex == firstEdge.VertexA || currentVertex == firstEdge.VertexB;
            if (firstEdgesContainsCurrentVertex && visitedEdges.Count > 1)
            {
                visitedEdges.Remove(visitedEdges.Last());
                return true;
            }

            List<Edge> possibleEdges = cycledSpanningTree.Edges.FindAll(
                (edge) => edge.VertexA == currentVertex || edge.VertexB == currentVertex
            );

            foreach(Edge edge in possibleEdges)
            {
                if (edge == lastEdge) continue; //to not go back immidately

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

            //Console.WriteLine($"The longest vertex is {maxEdge.VertexA.DistanceTo(maxEdge.VertexB)}\n");

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

        public Dictionary<WayPoint, WayPoint> FindMinimumWeightPerfectMatching(Graph fullGraph, Graph spanningTreeInducedGraph)
        {
            var matching = new Dictionary<WayPoint, WayPoint>();
            const string BlossomVExe = "BlossomV.exe";
            const string InputFile = "SpanningTree_input.txt";
            const string OutputFile = "PM_out.txt";

            WriteGraphToBlossom4(fullGraph, spanningTreeInducedGraph, InputFile);

            ProcessStartInfo startInfo = new();
            startInfo.FileName = BlossomVExe;
            startInfo.Arguments = $"-e {InputFile} -w {OutputFile}";

            using (Process process = Process.Start(startInfo))
            {
                process.WaitForExit();
            }

            string[] lines = File.ReadAllLines(OutputFile);

            foreach (string line in lines)
            {
                if (line == lines[0]) continue;
                string[] tokens = line.Split(" ");
                Debug.Assert(tokens.Length == 2);
                int v1 = int.Parse(tokens[0]);
                int v2 = int.Parse(tokens[1]);
                matching[spanningTreeInducedGraph.Vertices.ElementAt(v1)] = spanningTreeInducedGraph.Vertices.ElementAt(v2);
            }

            return matching;
        }

        private void WriteGraphToBlossom4(Graph fullGraph, Graph spanningTree, string inputFile)
        {
            using (StreamWriter file = new(inputFile))
            {
                file.WriteLine($"{spanningTree.Vertices.Count} {spanningTree.Edges.Count}");
                foreach(Edge edge in spanningTree.Edges)
                {
                    WayPoint vertexA = edge.VertexA;
                    WayPoint vertexB = edge.VertexB;
                    List<WayPoint> vertices = spanningTree.Vertices;
                    string line = $"{vertices.IndexOf(vertexA)} {vertices.IndexOf(vertexB)} {(int)(vertexA.DistanceTo(vertexB) * 100)}";
                    file.WriteLine(line);
                }
            }
        }

        private List<WayPoint> HierholzerEulerianTour(Graph g)
        {
            var eulerianTour = new List<WayPoint>();

            // Step 1: Create a copy of the graph
            Graph graphClone = new Graph(g);

            // Step 2: Ensure every vertex has an even degree
            foreach (var vertex in graphClone.Vertices)
            {
                if (graphClone.GetDegree(vertex) % 2 != 0)
                {
                    throw new InvalidOperationException("The graph does not have all vertices with even degree, thus it cannot have an Eulerian circuit.");
                }
            }

            // Step 3: Choose a start vertex with a non-zero degree
            WayPoint startVertex = graphClone.Vertices.FirstOrDefault(v => graphClone.GetDegree(v) > 0);

            if (startVertex != null)
            {
                var stack = new Stack<WayPoint>();
                stack.Push(startVertex);

                while (stack.Count > 0)
                {
                    var currentVertex = stack.Peek();
                    var nextVertex = graphClone.GetAdjacentVertex(currentVertex);

                    if (nextVertex == null)
                    {
                        eulerianTour.Add(stack.Pop());
                    }
                    else
                    {
                        //graphClone.RemoveEdge(new Edge(currentVertex, nextVertex));
                        stack.Push(nextVertex);
                    }
                }

                eulerianTour.Reverse();
            }
            else
            {
                throw new InvalidOperationException("The graph is empty or has no edges.");
            }

            return eulerianTour;
        }

        //private List<WayPoint> HierholzerEulerianTour(Graph g)
        //{
        //    var eulerianTour = new List<WayPoint>();

        //    Graph graphClone = new Graph(g);

        //    WayPoint startVertex = null;
        //    foreach(var vertex in g.Vertices)
        //    {
        //        if(graphClone.GetDegree(vertex) % 2 != 0)
        //        {
        //            startVertex = vertex;
        //            break;
        //        }
        //    }

        //    if(startVertex != null)
        //    {
        //        var stack = new Stack<WayPoint>();
        //        stack.Push(startVertex);

        //        while(stack.Count > 0)
        //        {
        //            var currentVertex = stack.Peek();
        //            var nextVertex = graphClone.GetAdjacentVertex(currentVertex);

        //            if(nextVertex == null)
        //            {
        //                eulerianTour.Add(stack.Pop());
        //            }
        //            else
        //            {
        //                graphClone.RemoveEdge(new Edge(currentVertex, nextVertex));
        //                stack.Push(nextVertex);
        //            }
        //        }

        //        eulerianTour.Reverse();
        //    }
        //    return eulerianTour;
        //}       

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

        private Graph CombineMatchingAndSpanningTree(Dictionary<WayPoint, WayPoint> matching, Graph MST)
        {
            Graph H = new Graph(MST);

            // Step 2: Add matching edges to the copy of MST
            foreach (var edge in matching)
            {
                WayPoint vertexA = edge.Key;
                WayPoint vertexB = edge.Value;

                H.AddEdgeMultiGraph(new Edge(vertexA, vertexB));
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
                Debug.Assert(vertexDegrees[vertex] % 2 == 0);
            }
            return H;
        }

        public override string? ToString()
        {
            return $"christofides_{mstAlgorithm.Method.Name}";
        }
    }
}
