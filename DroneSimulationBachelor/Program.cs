using DroneSimulationBachelor;
using DroneSimulationBachelor.Abstractions;
using DroneSimulationBachelor.Implementations;
using DroneSimulationBachelor.Model;
using ScottPlot.Statistics.Interpolation;
using System;
using System.Diagnostics;
using System.IO;
using System.Reflection.Metadata.Ecma335;
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
            //InitialiseCounterExampleTSP_DataSet_E
        };

        //foreach(var generator in dataGenerators)
        //{
        //    List<WayPoint> data = generator();
        //    GenerateResults2(data, $"DataSet_{dataGenerators.IndexOf(generator)}");
        //}

        List<List<WayPoint>> randomDataStack = new();
        List<List<WayPoint>> outlierDataStack = new();
        List<List<WayPoint>> clusterDataStack = new();
        const int SCENARIO_COUNT = 20;


        for(int creationIndex = 0; creationIndex < SCENARIO_COUNT; creationIndex++)
        {
            randomDataStack.Add(GenerateRectangularRandomDataSets(DateTime.MinValue, 10, 4));
            randomDataStack.Add(GenerateRectangularRandomDataSets(DateTime.MinValue, 30, 4));
            randomDataStack.Add(GenerateRectangularRandomDataSets(DateTime.MinValue, 50, 4));

            outlierDataStack.Add(GenerateRectangularOutlierDataSets(DateTime.MinValue, 10, 4));
            outlierDataStack.Add(GenerateRectangularOutlierDataSets(DateTime.MinValue, 30, 4));
            outlierDataStack.Add(GenerateRectangularOutlierDataSets(DateTime.MinValue, 50, 4));

            clusterDataStack.Add(GenerateRectangularClusteredDataSets(DateTime.MinValue, 10, 4));
            clusterDataStack.Add(GenerateRectangularClusteredDataSets(DateTime.MinValue, 30, 4));
            clusterDataStack.Add(GenerateRectangularClusteredDataSets(DateTime.MinValue, 50, 4));
        }

        //indexed at 1 for readability in directory structure:
        int dataSetNumber = 1;
        foreach(var route in randomDataStack)
        {
            Directory.CreateDirectory($"randomsData_{dataSetNumber}");
            JsonHandler.WriteToJson(Path.Combine($"randomsData_{dataSetNumber}", "data.json"), route);
            GenerateResults2(route, $"randomsData_{dataSetNumber++}");
        }
        dataSetNumber = 1;
        foreach (var route in outlierDataStack)
        {
            Directory.CreateDirectory($"outlier_{dataSetNumber}");
            JsonHandler.WriteToJson(Path.Combine($"outlier_{dataSetNumber}", "data.json"), route);
            GenerateResults2(route, $"outlier_{dataSetNumber++}");
        }
        dataSetNumber = 1;
        foreach (var route in clusterDataStack)
        {
            Directory.CreateDirectory($"cluster_{dataSetNumber}");
            JsonHandler.WriteToJson(Path.Combine($"cluster_{dataSetNumber}", "data.json"), route);
            GenerateResults2(route, $"cluster_{dataSetNumber++}");
        }

        //List<WayPoint> randomData = new();
        //List<WayPoint> notRandomData = InitialiseDummyDataSet_A();
        //randomData.Add(new CentralServer(0, 0));
        //randomData.AddRange(GenerateRectangularRandomDataSets(DateTime.MinValue, 30));

        //Directory.CreateDirectory("randomData_1");
        //JsonHandler.WriteToJson(Path.Combine("randomData_1", "data.json"),randomData);

        ////List<WayPoint> readRandomData = JsonHandler.ReadFromJson(Path.Combine("randomData_1", "data.json"));
        //GenerateResults2(randomData, "randomData_1");
    }


    private static void GenerateResults2(List<WayPoint> data, string dataName)
    {
        //Keep track of all maxReactionTimes
        double[] maxReactionTimes = new double[data.Count];

        //Create root Directory for entire DataSet
        Directory.CreateDirectory(dataName);

        //Create Directory inside root DataSet directory for the MST
        string mstPath = Path.Combine(dataName, "MST_Solution"); 
        Directory.CreateDirectory(mstPath);
        //Create routes and simulate the normal christofides with MST
        List<WayPoint> christofidesRoute = GenerateChristofidesRoute(data);

        double simulationPeriod = HamiltonianTourLength(christofidesRoute);
        int numberOfCycles = 3;

        int totalSimulationDuration = (int)(Math.Ceiling((numberOfCycles * simulationPeriod)));

        CentralServer ServerResults = SimulateRoute(christofidesRoute, totalSimulationDuration);
        WriteCSVandPlot(ServerResults, mstPath);

        maxReactionTimes[0] = CalculateMaxReactionTime(christofidesRoute);
        WriteMetaFile(christofidesRoute, mstPath);

        //Create all routes with enforced Edges and simulate christofides with modified Spanning Trees
        List<List<WayPoint>> modifiedCHristofidesRoutes = GenerateModifiedSpanningTreeChristofidesRoute(data);
        int modificationID = 1;

        double minimumReactionTime = -1;

        foreach(List<WayPoint> route in modifiedCHristofidesRoutes)
        {
            double maxReactionTime = CalculateMaxReactionTime(route);
            maxReactionTimes[modificationID] = maxReactionTime;

            string routeName = $"ChristofidesModifiedOnVertex_{modificationID++}";
            Directory.CreateDirectory(Path.Combine(dataName, routeName));
            WriteMetaFile(route, Path.Combine(dataName, routeName));

            ServerResults = SimulateRoute(route, 10);
            WriteCSVandPlot(ServerResults, Path.Combine(dataName, routeName));
        }

        using (StreamWriter file = new(Path.Combine(dataName, "minimumReactionTime.txt")))
        {
            List<double> maxReactionTimesList = maxReactionTimes.ToList();
            int smallestMaxReactionIndex = maxReactionTimesList.IndexOf(maxReactionTimes.Min());
            if(smallestMaxReactionIndex == 0)
            {
                file.Write("smallest MRT is the MST with route: ");
                foreach(var vertex in christofidesRoute)
                {
                    file.Write($"{vertex} ");
                }
                file.WriteLine($"\nTotal tour length of MST Route is: {HamiltonianTourLength(christofidesRoute)}");
            }
            else
            {
                List<WayPoint> smallestMRTRoute = new();
                if(smallestMaxReactionIndex == modifiedCHristofidesRoutes.Count)
                {
                    smallestMRTRoute = modifiedCHristofidesRoutes[smallestMaxReactionIndex - 1];
                }
                else
                {
                    smallestMRTRoute = modifiedCHristofidesRoutes[smallestMaxReactionIndex];
                }
                WayPoint first = smallestMRTRoute[0];
                WayPoint second = smallestMRTRoute[1];
                file.Write($"smallest MRT is the ST with with enforced edge: {first} - {second}");
                file.WriteLine($"\nTotal length of the tour with smallest MRT is: {HamiltonianTourLength(smallestMRTRoute)}");
            }
        }
        HistogramPlotter plotter = new HistogramPlotter();
        plotter.PlotMaxReactionTimesPicture(maxReactionTimes, 5, dataName);
    }

    private static void WriteMetaFile(List<WayPoint> route, string path)
    {
        using (StreamWriter file = new(Path.Combine(path, "meta.txt")))
        {
            foreach (var vertex in route)
            {
                file.Write($"{vertex} ");
            }
            file.WriteLine($"\nLength of entire route: {HamiltonianTourLength(route)}");
            file.WriteLine($"Maximum Reaction Time: {CalculateMaxReactionTime(route)}");
            file.WriteLine($"Length of the first edge: {route[0].DistanceTo(route[1])}");
        }
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
            Edge enforcedEdge = new Edge(X, vertex);
            Graph enforcedEdgeSpanningTree = Christofides.EnforcedEdgeSpanningTree(new Graph(MinimumSpanningTree),enforcedEdge);
            List<WayPoint> route = christofides.GenerateRoute(new Graph(data), enforcedEdgeSpanningTree, enforcedEdge);
            Debug.Assert(route.First() is CentralServer);
            Debug.Assert(route[1] == vertex);
            routes.Add(route);
        }
        return routes;
    }

    private static CentralServer SimulateRoute(List<WayPoint> route, int simulationDurationInSeconds)
    {
        Drone drone = new Drone(route, DateTime.MinValue);
        DateTime totalDurationInSeconds = DateTime.MinValue.AddSeconds(simulationDurationInSeconds);

        while (drone.CurrentTime <= totalDurationInSeconds)
        {
            drone.NextWayPoint();
        }
        CentralServer X = (CentralServer)route.Where(vertex => vertex is CentralServer).First();
        return X;
    }

    private static double CalculateMaxReactionTime(List<WayPoint> route)
    {
        double totalTourLength = HamiltonianTourLength(route);
        double restOfTourLength = totalTourLength - route[0].DistanceTo(route[1]);

        //Console.WriteLine($" total tour length: {totalTourLength} restOfTourLength: {restOfTourLength}");

        return CalculateMaxReactionTime(totalTourLength, restOfTourLength);
    }
    private static double CalculateMaxReactionTime(double totalTourLength, double restOfTourLength)
    {
        return totalTourLength + restOfTourLength;
    }

    private static void WriteCSVandPlot(CentralServer X, string directoryPath)
    {
        List<string> paths = new();
        foreach (var kvp in X.ReactionTimes)
        {
            string fileName = $"{kvp.Key}_distribution.csv";
            string filePath = Path.Combine(directoryPath, fileName);
            using (StreamWriter file = new(filePath))
            {
                file.Write(HistogramToCSVString(kvp.Value));
            }
            paths.Add(filePath);
        }
        HistogramPlotter plotter = new HistogramPlotter();
        plotter.PlotMultipleReactionTimeHistogram(paths.ToArray());
    }

    private static void printTour(List<WayPoint> tour)
    {
        foreach (WayPoint vertex in tour)
        {
            if (vertex is DataNode)
            {
                DataNode vert = (DataNode)vertex;
                //Console.WriteLine(vert.ID);
            }
            //else if (vertex is CentralServer) Console.WriteLine("Central Server!");
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

    private static double HamiltonianTourLength(List<WayPoint> tour)
    {
        if (tour.First() != tour.Last()) return -1;

        double tourLength = 0;

        for(int currentIndex = 0; currentIndex < tour.Count-1; currentIndex++)
        {
            double distance = tour[currentIndex].DistanceTo(tour[currentIndex + 1]);
            tourLength += distance;
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


    //Scenarios
    public static List<WayPoint> GenerateRectangularRandomDataSets(DateTime startTime, int countDataNodes,
        int fixedDataNodePeriod = -1, int xBounds = 100, int yBounds = 100)
    {
        Random random = new Random();
        List<WayPoint> dataPoints = new List<WayPoint>();
        dataPoints.Add(new CentralServer(xBounds / 2, yBounds / 2));
        for (int i = 0; i < countDataNodes; i++)
        {
            int x = random.Next(0, xBounds);
            int y = random.Next(0, yBounds);
            TimeSpan period = new TimeSpan();
            if (fixedDataNodePeriod == -1)
            {
                period = TimeSpan.FromSeconds(random.Next(1, 5));
            }
            else
            {
                period = TimeSpan.FromSeconds(fixedDataNodePeriod);
            }
            string id = Guid.NewGuid().ToString().Substring(0,4); // Generate a unique ID
            dataPoints.Add(new DataNode(x, y, period, startTime, id));
        }
        return dataPoints;
    }
    public static List<WayPoint> GenerateRectangularOutlierDataSets(DateTime startTime, int countDataNodes,
    int fixedDataNodePeriod = -1, int xBounds = 100, int yBounds = 100, int outlierCount = 1, int outlierFactor = 5)
    {
        Random random = new Random();
        List<WayPoint> dataPoints = new List<WayPoint>();
        dataPoints.Add(new CentralServer(xBounds / 2, yBounds / 2));
        // Define closer range boundaries for normal points - outlierFactor is the denominator, so the smaller it is the smaller is the area that is considered "normal points"
        int closeRangeX = xBounds / outlierFactor;
        int closeRangeY = yBounds / outlierFactor;

        // Generate normal points
        for (int i = 0; i < countDataNodes - outlierCount; i++)
        {
            int x = random.Next(0, closeRangeX);
            int y = random.Next(0, closeRangeY);
            TimeSpan period = new TimeSpan();
            if (fixedDataNodePeriod == -1)
            {
                period = TimeSpan.FromSeconds(random.Next(1, 5));
            }
            else
            {
                period = TimeSpan.FromSeconds(fixedDataNodePeriod);
            }
            string id = Guid.NewGuid().ToString().Substring(0, 5); // Generate a unique ID
            dataPoints.Add(new DataNode(x, y, period, startTime, id));
        }

        // Generate outliers
        for (int i = 0; i < outlierCount; i++)
        {
            int x = random.Next(closeRangeX, xBounds);
            int y = random.Next(closeRangeY, yBounds);
            TimeSpan period = new TimeSpan();
            if (fixedDataNodePeriod == -1)
            {
                period = TimeSpan.FromSeconds(random.Next(1, 5));
            }
            else
            {
                period = TimeSpan.FromSeconds(fixedDataNodePeriod);
            }
            string id = Guid.NewGuid().ToString().Substring(0, 5); // Generate a unique ID
            dataPoints.Add(new DataNode(x, y, period, startTime, id));
        }

        return dataPoints;
    }
    public static List<WayPoint> GenerateRectangularClusteredDataSets(DateTime startTime, int countDataNodes,
    int clusterCount, int fixedDataNodePeriod = -1, int xBounds = 100, int yBounds = 100, int clusterBoundary = 10)
    {
        Random random = new Random();
        List<WayPoint> dataPoints = new List<WayPoint>();
        List<(int x, int y)> clusterCenters = new List<(int x, int y)>();
        dataPoints.Add(new CentralServer(xBounds / 2, yBounds / 2));
        // Determine cluster centers
        for (int i = 0; i < clusterCount; i++)
        {
            int clusterX = random.Next(0, xBounds);
            int clusterY = random.Next(0, yBounds);
            clusterCenters.Add((clusterX, clusterY));
        }

        // Distribute points among clusters
        int pointsPerCluster = countDataNodes / clusterCount;
        int remainingPoints = countDataNodes % clusterCount;

        for (int i = 0; i < clusterCount; i++)
        {
            int pointsInThisCluster = pointsPerCluster + (i < remainingPoints ? 1 : 0);
            for (int j = 0; j < pointsInThisCluster; j++)
            {
                // Generate points around the cluster center within a small boundary
                int x = clusterCenters[i].x + random.Next(-xBounds / clusterBoundary, xBounds / clusterBoundary);
                int y = clusterCenters[i].y + random.Next(-yBounds / clusterBoundary, yBounds / clusterBoundary);

                // Ensure points are within bounds
                x = Math.Max(0, Math.Min(x, xBounds));
                y = Math.Max(0, Math.Min(y, yBounds));

                TimeSpan period = new TimeSpan();
                if (fixedDataNodePeriod == -1)
                {
                    period = TimeSpan.FromSeconds(random.Next(1, 5));
                }
                else
                {
                    period = TimeSpan.FromSeconds(fixedDataNodePeriod);
                }
                string id = Guid.NewGuid().ToString().Substring(0, 5); // Generate a unique ID
                dataPoints.Add(new DataNode(x, y, period, startTime, id));
            }
        }

        return dataPoints;
    }
}