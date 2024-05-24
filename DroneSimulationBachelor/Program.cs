using DroneSimulationBachelor;
using DroneSimulationBachelor.Abstractions;
using DroneSimulationBachelor.Implementations;
using DroneSimulationBachelor.Model;

class Program
{
    static void Main(string[] args)
    {
        List<WayPoint> christofidesTour = InitialiseDummyDataSet_A(new Christofides());
        List<WayPoint> modifiedChristofidesTour = InitialiseDummyDataSet_A(new Christofides(Christofides.ModifiedMST));

        double christofidesTourLength = HamiltonianTourLength(christofidesTour);
        double modifiedChristofidesTourLength = HamiltonianTourLength(modifiedChristofidesTour);

        double maxReactionTimeChristofide = CalculateMaxReactionTime(christofidesTourLength, christofidesTourLength - christofidesTour[0].DistanceTo(christofidesTour[1]));
        double maxReactionTimeModified = CalculateMaxReactionTime(modifiedChristofidesTourLength, modifiedChristofidesTourLength - modifiedChristofidesTour[0].DistanceTo(modifiedChristofidesTour[1]));
        Console.WriteLine($"the entire tour length of {nameof(christofidesTour)}is: {christofidesTourLength}");
        Console.WriteLine($"the entire tour length of {nameof(modifiedChristofidesTour)}is: {modifiedChristofidesTourLength}");

        Console.WriteLine($"the maximum Reaction Time is {maxReactionTimeChristofide}");
        Console.WriteLine($"the maximum Reaction Time is {maxReactionTimeModified}");

        printTour(christofidesTour);
        Console.WriteLine("---------------");
        printTour(modifiedChristofidesTour);

        Drone drone_A = new(christofidesTour, DateTime.MinValue);
        //timesteps vs time
        while (drone_A.CurrentTime <= DateTime.MinValue.AddMinutes(10))
        {
            drone_A.NextWayPoint();
        }

        Drone drone_B = new(modifiedChristofidesTour, DateTime.MinValue);
        while (drone_B.CurrentTime <= DateTime.MinValue.AddMinutes(10))
        {
            drone_B.NextWayPoint();
        }

        List<string> paths = new();

        CentralServer X = (CentralServer)christofidesTour.Where(vertex => vertex is CentralServer).First();
        WriteCSVandPlot(paths, X, "Christofides");
        X = (CentralServer)modifiedChristofidesTour.Where(vertex => vertex is CentralServer).First();
        WriteCSVandPlot(paths, X, "Modified Christofides");
    }

    private static double CalculateMaxReactionTime(double totalTourLength, double restOfTourLength)
    {
        return totalTourLength + restOfTourLength;
    }

    private static void WriteCSVandPlot(List<string> paths, CentralServer X, string usedRouteGen)
    {
        foreach (var kvp in X.ReactionTimes)
        {
            //using (StreamWriter file = new($"{routeGen.GetType().Name}_{kvp.Key.Substring(0,4)}_distribution.csv"))
            string filename = $"{kvp.Key}_distribution_{usedRouteGen}.csv";
            using (StreamWriter file = new(filename))
            {
                file.Write(HistogramToCSVString(kvp.Value));
            }
            paths.Add(filename);
        }
        HistogramPlotter plotter = new HistogramPlotter();
        plotter.PlotHistogram(paths.ToArray(), suffix: "_" + usedRouteGen);
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

    private static List<WayPoint> InitialiseDummyDataSet_A(IRouteGenerator routeGen)
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
        
        List<WayPoint> tour = routeGen.GenerateRoute(points);
        return tour;
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
                tourLength += tour[currentIndex].DistanceTo(tour[currentIndex + 1]);
            }
        }
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