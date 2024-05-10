using DroneSimulationBachelor.Abstractions;

public class Graph
{
    public List<Edge> Edges { get; private set; }
    public List<WayPoint> Vertices;
    

    public Graph()
    {
        Edges = new();
        Vertices = new();
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

    public void AddEdge(Edge edge)
    {
        if(!Edges.Contains(edge))
        {
            Edges.Add(edge);
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
}