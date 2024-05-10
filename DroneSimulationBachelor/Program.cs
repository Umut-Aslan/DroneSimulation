using DroneSimulationBachelor;
using DroneSimulationBachelor.Abstractions;
using DroneSimulationBachelor.Implementations;
using DroneSimulationBachelor.Model;

class Program
{
    static void Main(string[] args)
    {
        DateTime startTime = DateTime.MinValue;
        DataNode d1 = new(0, 5, TimeSpan.FromMinutes(2.0), startTime, "A");
        DataNode d2 = new(5, 5, TimeSpan.FromMinutes(5.0), startTime, "B");
        DataNode d3 = new(5, 0, TimeSpan.FromMinutes(1.0), startTime, "C");
        DataNode d4 = new(10, 10, TimeSpan.FromMinutes(8.0), startTime, "D");
        DataNode d5 = new(-5, 8, TimeSpan.FromMinutes(5.0), startTime, "E");
        DataNode d6 = new(-12, -3, TimeSpan.FromMinutes(4.0), startTime, "F");
        DataNode d7 = new(19, 3, TimeSpan.FromMinutes(4.0), startTime, "G");
        DataNode d8 = new(4, -8, TimeSpan.FromMinutes(13.0), startTime, "H");
        DataNode d9 = new(3, -2, TimeSpan.FromMinutes(6.0), startTime, "I");
        DataNode d10 = new(-5, 15, TimeSpan.FromMinutes(1.0), startTime, "J");
        DataNode d11 = new(15, -7, TimeSpan.FromMinutes(8.0), startTime, "K");


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

        List<WayPoint> points = new List<WayPoint>() { X, d1, d2, d3, d4, d5, d6, d7, d8, d9, d10, d11 };
        IRouteGenerator christofides = new Christofides();
        List<WayPoint> tour = christofides.GenerateRoute(points);

        foreach(WayPoint vertex in tour)
        {
            if (vertex is DataNode)
            {
                DataNode vert = (DataNode)vertex;
                Console.WriteLine(vert.ID);
            }
            else if(vertex is CentralServer) Console.WriteLine("Central Server!");
        }

        Drone drone = new(tour, startTime);
        //timesteps vs time
        while (drone.CurrentTime <= startTime.AddMinutes(10000))
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

            HistogramPlotter plotter = new HistogramPlotter();
            plotter.PlotHistogram($"{kvp.Key}_distribution.csv");
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