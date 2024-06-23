using DroneSimulationBachelor.Abstractions;
using DroneSimulationBachelor.Implementations;

public class Graph
{
    public List<Edge> Edges { get; private set; }
    public List<WayPoint> Vertices;
    public Graph()
    {
        Edges = new();
        Vertices = new();
    }

    public Graph(Graph toCopy)
    {
        Edges = new(toCopy.Edges);
        Vertices = new(toCopy.Vertices);
    }

    public Graph(List<WayPoint> dataPoints)
    {
        Edges = new List<Edge>();
        Vertices = new List<WayPoint>();

        foreach (WayPoint p in dataPoints) {
            Vertices.Add(p);
            foreach(WayPoint q in dataPoints)
            {
                if(p != q)
                {
                    Edge edge = new Edge(p, q);
                    AddEdge(edge);
                }
            }
        }
    }

    public virtual void AddEdge(Edge edge)
    {
        if(!Edges.Contains(edge))
        {
            Edges.Add(edge);
            if(!Vertices.Contains(edge.VertexA)) Vertices.Add(edge.VertexA);
            if(!Vertices.Contains(edge.VertexB)) Vertices.Add(edge.VertexB);
        }
    }

    public void RemoveEdge(Edge edge) 
    {
        Edges.RemoveAll(e => e.Equals(edge));
    }

    public WayPoint GetAdjacentVertex(WayPoint vertex)
    {
        foreach (var edge in Edges)
        {
            if(edge.VertexA == vertex)
            {
                Edges.Remove(edge);
                return edge.VertexB;
            }
            else if(edge.VertexB == vertex)
            {
                Edges.Remove(edge);
                return edge.VertexA;
            }
        }
        return null;
    }

    public int GetDegree(WayPoint vertex)
    {
        return Edges.Count(edge => edge.VertexA == vertex || edge.VertexB == vertex);
    }

    public BlossomContraction Contract(Graph blossom)
    {
        BlossomContraction v_b = new BlossomContraction();

        foreach(WayPoint vertex in blossom.Vertices)
        {
            Vertices.Remove(vertex);
            foreach (Edge edge in Edges)
            {
                if (edge.Contains(vertex))
                {
                    if(!blossom.Edges.Contains(edge))
                        AddEdge(new Edge(v_b, vertex));
                    RemoveEdge(edge);
                }
            }
        }
        return v_b;
    }
}