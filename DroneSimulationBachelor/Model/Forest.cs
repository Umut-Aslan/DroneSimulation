using DroneSimulationBachelor.Abstractions;
using ScottPlot.MinMaxSearchStrategies;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.Versioning;
using System.Text;
using System.Threading.Tasks;

namespace DroneSimulationBachelor.Model
{
    public class Forest : Graph
    {
        Dictionary<WayPoint, WayPoint> VertexRoot;
        Dictionary<WayPoint, bool> VertexIsMarked;
        Dictionary<WayPoint, int> VertexDepth;
        Dictionary<WayPoint, WayPoint> VertexParent;
        public Forest()
        {

        }
        public void AddConnectedEdge(WayPoint parent, WayPoint child)
        {
            if (!Vertices.Contains(parent)) throw new Exception("Added edge should be connected to existing vertex => parent has to exist");
            base.AddEdge(new Edge(parent, child));
            VertexParent[child] = parent;
            VertexDepth[child] = VertexDepth[parent] + 1;
            VertexRoot[child] = VertexRoot[parent];
            VertexIsMarked[child] = false;
        }

        public List<WayPoint> GetPathToRoot(WayPoint v)
        {
            List<WayPoint> path = new() { v };
            WayPoint currentVertex = v;
            
            while(currentVertex != VertexRoot[v])
            {
                currentVertex = VertexParent[currentVertex];
                path.Add(currentVertex);
            }
            return path;
        }
        public void MarkVertex(WayPoint vertex) => VertexIsMarked[vertex] = true;
        public WayPoint GetRoot(WayPoint vertex) => VertexRoot[vertex]; 
        public WayPoint GetParent(WayPoint vertex) => VertexParent[vertex];

        public int GetDepth(WayPoint vertex) => VertexDepth[vertex];

        public WayPoint GetUnmarkedVertexWithEvenDepth()
        {
            foreach(WayPoint vertex in Vertices)
            {
                if (VertexIsMarked[vertex]) continue;
                if (VertexDepth[vertex] % 2 == 0)
                {
                    return vertex;
                }
            }
            return null;
        }

        public void AddRoots(HashSet<WayPoint> newRoots)
        {
            foreach (WayPoint vertex in newRoots)
            {
                VertexRoot[vertex] = vertex;
                VertexIsMarked[vertex] = true;
                VertexDepth[vertex] = 0;
                VertexParent[vertex] = null;
                Vertices.Add(vertex);
            }
        }
    }
}
