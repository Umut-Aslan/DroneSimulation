using DroneSimulationBachelor.Abstractions;
using System.Security;

public class Edge
{
    public WayPoint VertexA { get; set; }
    public WayPoint VertexB { get; set; }
    public double Distance { get; set; }

    public Edge(WayPoint vertexA, WayPoint vertexB)
    {
        VertexA = vertexA;
        VertexB = vertexB;
        Distance = VertexA.DistanceTo(VertexB);
    }

    public Edge()
    {
    }

    //public override bool Equals(object? other)
    //{
    //    if (other is null) return false;
    //    if (other is not Edge) return false;

    //    Edge otherEdge = (Edge)other!;

    //    return (this.VertexA == otherEdge.VertexA && this.VertexB == otherEdge.VertexB) ||
    //        (this.VertexA == otherEdge.VertexB && this.VertexB == otherEdge.VertexA);
    //}

    public override bool Equals(object obj)
    {
        if (obj == null || GetType() != obj.GetType())
        {
            return false;
        }

        var otherEdge = (Edge)obj;
        return (VertexA == otherEdge.VertexA && VertexB == otherEdge.VertexB) ||
               (VertexA == otherEdge.VertexB && VertexB == otherEdge.VertexA);
    }

    public override int GetHashCode()
    {
        return VertexA.GetHashCode() ^ VertexB.GetHashCode();
    }

    public static bool operator ==(Edge? left, Edge? right)
    {
        return EqualityComparer<Edge>.Default.Equals(left, right);
    }

    public static bool operator !=(Edge? left, Edge? right)
    {
        return !(left == right);
    }
}