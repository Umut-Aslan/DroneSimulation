using DroneSimulationBachelor.Abstractions;

public class Graph
{
    public Dictionary<WayPoint, List<Edge>> AdjacencyList;

    public Graph()
    {
        AdjacencyList = new Dictionary<WayPoint, List<Edge>>();
    }

    public Graph(List<WayPoint> dataPoints)
    {
        AdjacencyList = new Dictionary<WayPoint, List<Edge>>();

        foreach(WayPoint p in dataPoints) {
            AddVertex(p);
            foreach(WayPoint q in dataPoints)
            {
                if(!(p.X == q.X || p.Y == q.Y))
                {
                    AddEdge(p, q, p.DistanceTo(q));
                }
            }
        }
    }

    public void AddVertex(WayPoint vertex)
    {
        if (!AdjacencyList.ContainsKey(vertex))
        {
            AdjacencyList[vertex] = new List<Edge>();
        }
    }

    public WayPoint? RemoveVertex(WayPoint vertex)
    {
        if (AdjacencyList.ContainsKey(vertex))
        {
            WayPoint ToRemove = vertex;
            AdjacencyList.Remove(vertex);
            return ToRemove;
        }
        return null;
    }

    public void AddEdge(WayPoint source, WayPoint destination, double weight)
    {
        AdjacencyList[source].Add(new Edge(destination, weight));
        AdjacencyList[destination].Add(new Edge(source, weight)); // Since it's an undirected graph
    }
}