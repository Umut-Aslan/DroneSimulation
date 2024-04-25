using DroneSimulationBachelor;

class Program
{
    static void Main(string[] args)
    {
        DateTime startTime = DateTime.MinValue;
        DataNode d1 = new(0, 5, TimeSpan.FromMinutes(4.0), startTime, "A");
        DataNode d2 = new(5, 5, TimeSpan.FromMinutes(4.0), startTime, "B");
        DataNode d3 = new(5, 0, TimeSpan.FromMinutes(4.0), startTime, "C");

        //List<WayPoint> nodes = new();
        CentralServer X = new(0, 0);
        //nodes.Add(X);
        //List<WayPoint> dataNodes = GenerateRandomDataPoints(startTime, 50);
        //foreach(WayPoint node in dataNodes)
        //{
        //    nodes.Add(node);
        //}

        //IRouteGenerator routeGen = new NearestNeighbourRouteGenerator();
        //IRouteGenerator routeGen = new SimulatedAnnealingRouteGenerator();
        //IRouteGenerator routeGen = new MinimumSpanningTreeRouteGenerator();
        //List<WayPoint> route = routeGen.GenerateRoute(nodes);

        List<WayPoint> route = new List<WayPoint>() { X, d1, d2, d3 };
        Drone drone = new(route, startTime);
        //timesteps vs time
        while (drone.CurrentTime <= startTime.AddMinutes(150))
        {
            drone.NextWayPoint();
        }

        foreach (var kvp in X.ReactionTimes)
        {
            //using (StreamWriter file = new($"{routeGen.GetType().Name}_{kvp.Key.Substring(0,4)}_distribution.csv"))
            using (StreamWriter file = new($"{kvp.Key}_distribution.csv"))
            {
                file.Write(HistogramToCSVString(kvp.Value));
            }
        }
    }

    static string HistogramToCSVString(Dictionary<TimeSpan, uint> data)
    {
        string result = "reaction_time;count\n";
        foreach (var kvp in data)
        {
            result += $"{kvp.Key};{kvp.Value}\n";
        }
        return result;
    }

    public static List<WayPoint> GenerateRandomDataPoints(DateTime startTime, int count)
    {
        Random random = new Random(1234);
        List<WayPoint> dataPoints = new List<WayPoint>();

        for (int i = 0; i < count; i++)
        {
            int x = random.Next(0, 100); // Assuming x and y are within range 0-100
            int y = random.Next(0, 100);
            TimeSpan period = TimeSpan.FromMinutes(random.Next(1, 5)); // Random transmission period between 1 minute and 1 day
            string id = Guid.NewGuid().ToString(); // Generate a unique ID
            dataPoints.Add(new DataNode(x, y, period, startTime, id));
        }
        return dataPoints;
    }
}