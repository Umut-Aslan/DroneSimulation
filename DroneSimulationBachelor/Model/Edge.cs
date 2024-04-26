using DroneSimulationBachelor.Abstractions;

public class Edge
{
    public WayPoint Destination { get; }
    public double Weight { get; }

    public Edge(WayPoint destination, double weight)
    {
        Destination = destination;
        Weight = weight;
    }
}