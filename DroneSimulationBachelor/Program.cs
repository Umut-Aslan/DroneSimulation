using DroneSimulationBachelor;
using DroneSimulationBachelor.Abstractions;
using DroneSimulationBachelor.Implementations;
using DroneSimulationBachelor.Model;
using ScottPlot.Statistics.Interpolation;
using System.Diagnostics;
using static DroneSimulationBachelor.Implementations.Christofides;

class Program
{
    private delegate List<WayPoint> WayPointGenerator();

    static void Main(string[] args)
    {
        List<WayPointGenerator> dataGenerators = new() {
            InitialiseDummyDataSet_A, 
            InitialiseClusteredDataSet_B, 
            InitialiseStarShapedDataSet_C,
            InitialiseLinearGraphDataSet_D,
            InitialiseCounterExampleTSP_DataSet_E,
        };
        foreach(var generator in dataGenerators)
        {
            List<WayPoint> data = generator();
            GenerateResults(data);
        }
    }

    private static void GenerateResults(List<WayPoint> data)
    {
        double[] maxReactionTimes = new double[data.Count];
        List<WayPoint> christofidesRoute = GenerateChristofidesRoute(data);
        maxReactionTimes[0] = CalculateMaxReactionTime(christofidesRoute);
        List<List<WayPoint>> modifiedChristofidesRoutes = GenerateModifiedSpanningTreeChristofidesRoute(data);

        SimulateRoute(christofidesRoute, 10, "Christofides", "DummySetA");
        printRouteInformation(christofidesRoute, "Christofides");

        int modificationID = 1;
        foreach (List<WayPoint> route in modifiedChristofidesRoutes)
        {
            maxReactionTimes[modificationID] = CalculateMaxReactionTime(route);
            string routeName = $"ChristofidesModifiedOnVertex_{modificationID++}";
            SimulateRoute(route, 10, routeName, "DummySetA");
            printRouteInformation(route, routeName);
        }
        HistogramPlotter plotter = new HistogramPlotter();
        plotter.PlotMaxReactionTimesPicture(maxReactionTimes, 5);
    }

    private static void printRouteInformation(List<WayPoint> route, string routeName)
    {
        double tourLength = HamiltonianTourLength(route);
        double firstEdgeLength = route[0].DistanceTo(route[1]);
        double maxReactionTime = CalculateMaxReactionTime(tourLength, tourLength - firstEdgeLength);
        Console.WriteLine($"The current route is: {routeName}");
        Console.WriteLine($"The length of the entire route is: {tourLength}");
        Console.WriteLine($"The maximum Reaction Time is {maxReactionTime}");
        Console.WriteLine("The tour goes like this: ");
        printTour(route);
        Console.WriteLine("---------------");
    }
    private static List<WayPoint> GenerateChristofidesRoute(List<WayPoint> data)
    {
        Christofides christofides = new Christofides();
        List<WayPoint> route = christofides.GenerateRoute(data);

        return route;
    }
    private static List<List<WayPoint>> GenerateModifiedSpanningTreeChristofidesRoute(List<WayPoint> data)
    {
        List<List<WayPoint>> routes = new List<List<WayPoint>>();
        Christofides christofides = new Christofides();
        CentralServer X = (CentralServer)data.Where(vertex => vertex is CentralServer).First();
        Graph MinimumSpanningTree = Christofides.MinimumSpanningTree(new Graph(data));

        foreach (WayPoint vertex in data)
        {
            if (vertex is CentralServer) continue;
            Graph enforcedEdgeSpanningTree = Christofides.EnforcedEdgeSpanningTree(new Graph(MinimumSpanningTree), new Edge(X, vertex));
            List<WayPoint> route = christofides.GenerateRoute(new Graph(data), enforcedEdgeSpanningTree);
            Debug.Assert(route.First() is CentralServer);
            Debug.Assert(route[1] == vertex);
            routes.Add(route);
        }

        return routes;
    }

    private static void SimulateRoute(List<WayPoint> route, int simulationDurationInMinutes, string routeName, string dataSetName)
    {
        Drone drone = new Drone(route, DateTime.MinValue);

        while(drone.CurrentTime <= DateTime.MinValue.AddMinutes(simulationDurationInMinutes))
        {
            drone.NextWayPoint();
        }

        List<string> paths = new();

        CentralServer X = (CentralServer)route.Where(vertex => vertex is CentralServer).First();
        WriteCSVandPlot(paths, X, $"{routeName}", dataSetName);
    }

    private static double CalculateMaxReactionTime(List<WayPoint> route)
    {
        double totalTourLength = HamiltonianTourLength(route);
        double restOfTourLength = totalTourLength - route[0].DistanceTo(route[1]);

        return CalculateMaxReactionTime(totalTourLength, restOfTourLength);
    }
    private static double CalculateMaxReactionTime(double totalTourLength, double restOfTourLength)
    {
        return totalTourLength + restOfTourLength;
    }

    private static void WriteCSVandPlot(List<string> paths, CentralServer X, string routeGenName, string dataSetName)
    {
        foreach (var kvp in X.ReactionTimes)
        {
            Directory.CreateDirectory(dataSetName);
            //using (StreamWriter file = new($"{routeGen.GetType().Name}_{kvp.Key.Substring(0,4)}_distribution.csv"))
            string fileName = $"{kvp.Key}_distribution_{routeGenName}.csv";
            string filePath = Path.Combine(dataSetName, fileName);
            using (StreamWriter file = new(filePath))
            {
                file.Write(HistogramToCSVString(kvp.Value));
            }
            paths.Add(filePath  );
        }
        HistogramPlotter plotter = new HistogramPlotter();
        plotter.PlotMultipleReactionTImeHistogram(paths.ToArray(), suffix: "_" + routeGenName);
    }

    private static void printTour(List<WayPoint> tour)
    {
        foreach (WayPoint vertex in tour)
        {
            if (vertex is DataNode)
            {
                DataNode vert = (DataNode)vertex;
                Console.WriteLine(vert.ID);
            }
            else if (vertex is CentralServer) Console.WriteLine("Central Server!");
        }
    }

    private static List<WayPoint> InitialiseDummyDataSet_A()
    {
        DateTime startTime = DateTime.MinValue;
        DataNode d1 = new(0, 5, TimeSpan.FromSeconds(4.0), startTime, "A");
        DataNode d2 = new(5, 5, TimeSpan.FromSeconds(4.0), startTime, "B");
        DataNode d3 = new(5, 0, TimeSpan.FromSeconds(4.0), startTime, "C");
        DataNode d4 = new(10, 10, TimeSpan.FromSeconds(4.0), startTime, "D");
        DataNode d5 = new(-5, 8, TimeSpan.FromSeconds(4.0), startTime, "E");
        DataNode d6 = new(-12, -3, TimeSpan.FromSeconds(4.0), startTime, "F");
        DataNode d7 = new(19, 3, TimeSpan.FromSeconds(4.0), startTime, "G");
        DataNode d8 = new(4, -8, TimeSpan.FromSeconds(4.0), startTime, "H");
        DataNode d9 = new(3, -2, TimeSpan.FromSeconds(4.0), startTime, "I");
        DataNode d10 = new(-5, 15, TimeSpan.FromSeconds(4.0), startTime, "J");
        DataNode d11 = new(15, -7, TimeSpan.FromSeconds(4.0), startTime, "K");

        CentralServer X = new(0, 0);
        List<WayPoint> points = new List<WayPoint>() { X, d1, d2, d3, d4, d5, d6, d7, d8, d9, d10, d11 };

        return points;
    }
    private static List<WayPoint> InitialiseClusteredDataSet_B()
    {
        DateTime startTime = DateTime.MinValue;
        CentralServer X = new(0, 0);

        //Cluster 1
        DataNode d1 = new(6, 2, TimeSpan.FromSeconds(4.0), startTime, "G");
        DataNode d2 = new(10, 3, TimeSpan.FromSeconds(4.0), startTime, "D");
        DataNode d3 = new(10, 6, TimeSpan.FromSeconds(4.0), startTime, "C");
        DataNode d4 = new(8, 5, TimeSpan.FromSeconds(4.0), startTime, "E");
        DataNode d5 = new(7, 5, TimeSpan.FromSeconds(4.0), startTime, "F");
        DataNode d6 = new(6, 6, TimeSpan.FromSeconds(4.0), startTime, "A");
        DataNode d7 = new(9, 8, TimeSpan.FromSeconds(4.0), startTime, "B");

        //Cluster 2
        DataNode d8 = new(0, -5, TimeSpan.FromSeconds(4.0), startTime, "H");
        DataNode d9 = new(-3, -7, TimeSpan.FromSeconds(4.0), startTime, "I");
        DataNode d10 = new(-11, -15, TimeSpan.FromSeconds(4.0), startTime, "J");
        DataNode d11 = new(-8, -11, TimeSpan.FromSeconds(4.0), startTime, "K");
        DataNode d12 = new(-4, -11, TimeSpan.FromSeconds(4.0), startTime, "L");
        DataNode d13 = new(-6, -12, TimeSpan.FromSeconds(4.0), startTime, "M");

        //Cluster 3
        DataNode d14 = new(-8, 5, TimeSpan.FromSeconds(4.0), startTime, "N");
        DataNode d15 = new(-8, 15, TimeSpan.FromSeconds(4.0), startTime, "Q");
        DataNode d16 = new(-12, 11, TimeSpan.FromSeconds(4.0), startTime, "R");
        DataNode d17 = new(-12, 7, TimeSpan.FromSeconds(4.0), startTime, "S");
        DataNode d18 = new(-5, 11, TimeSpan.FromSeconds(4.0), startTime, "P");
        DataNode d19 = new(-5, 7, TimeSpan.FromSeconds(4.0), startTime, "O");

        return new List<WayPoint>() { X, d1, d2, d3, d4, d5, d6, d7, d8, d9, d10, d11, d12, d13, d14, d15, d16, d17, d18, d19 };
    }
    private static List<WayPoint> InitialiseStarShapedDataSet_C()
    {
        DateTime startTime = DateTime.MinValue;

        CentralServer X = new(0, 0);

        DataNode d1 = new DataNode(0, 6, TimeSpan.FromSeconds(4.0), startTime, "A");
        DataNode d2 = new DataNode(4, 2, TimeSpan.FromSeconds(4.0), startTime, "B");
        DataNode d3 = new DataNode(4, -5, TimeSpan.FromSeconds(4.0), startTime, "C");
        DataNode d4 = new DataNode(-4, -5, TimeSpan.FromSeconds(4.0), startTime, "D");
        DataNode d5 = new DataNode(-4, 2, TimeSpan.FromSeconds(4.0), startTime, "E");

        return new List<WayPoint>() { X, d1, d2, d3, d4, d5 };

    }
    private static List<WayPoint> InitialiseLinearGraphDataSet_D()
    {
        DateTime startTime = DateTime.MinValue;
        CentralServer X = new(0, 0);
        DataNode d1 = new(1, 0, TimeSpan.FromSeconds(4.0), startTime, "A");
        DataNode d2 = new(2, 0, TimeSpan.FromSeconds(4.0), startTime, "B");
        DataNode d3 = new(3, 0, TimeSpan.FromSeconds(4.0), startTime, "C");
        DataNode d4 = new(4, 0, TimeSpan.FromSeconds(4.0), startTime, "D");
        DataNode d5 = new(5, 0, TimeSpan.FromSeconds(4.0), startTime, "E");
        DataNode d6 = new(6, 0, TimeSpan.FromSeconds(4.0), startTime, "F");
        DataNode d7 = new(7, 0, TimeSpan.FromSeconds(4.0), startTime, "G");

        return new List<WayPoint>() { X, d1, d2, d3, d4, d5, d6, d7 };
    }
    private static List<WayPoint> InitialiseCounterExampleTSP_DataSet_E()
    {
        throw new NotImplementedException();
    }


    private static double EulerianTourLength(List<WayPoint> tour)
    {
        if (tour[0] == tour[tour.Count - 1]) return HamiltonianTourLength(tour);
        else
        {
            List<WayPoint> hamiltonianTour = new List<WayPoint>(tour);
            hamiltonianTour.Add(tour[0]);
            return HamiltonianTourLength(tour);
        }
    }

    private static double HamiltonianTourLength(List<WayPoint> tour)
    {
        if (tour[0] != tour[tour.Count - 1]) return EulerianTourLength(tour);

        double tourLength = 0;

        for(int currentIndex = 0; currentIndex < tour.Count; currentIndex++)
        {
            if(currentIndex < tour.Count - 1)
            {
                double distance = tour[currentIndex].DistanceTo(tour[currentIndex + 1]);
                Console.WriteLine($"From Vertex: {tour[currentIndex]} to Vertex: {tour[currentIndex+1]} the Distance is {distance}\n");
                tourLength += distance;
            }
        }
        Console.WriteLine("----------------\n");
        return tourLength;
    }

    static string HistogramToCSVString(Dictionary<TimeSpan, uint> data)
    {
        string result = "reaction_time;count\n";
        foreach (var kvp in data)
        {
            result += $"{kvp.Key.TotalSeconds};{kvp.Value}\n";
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