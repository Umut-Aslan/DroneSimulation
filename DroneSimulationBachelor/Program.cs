using DroneSimulationBachelor;
using DroneSimulationBachelor.Abstractions;
using DroneSimulationBachelor.Implementations;
using DroneSimulationBachelor.Model;
using ScottPlot.Statistics.Interpolation;
using System;
using System.Data;
using System.Diagnostics;
using System.IO;
using System.Reflection.Metadata.Ecma335;
using System.Runtime.ExceptionServices;
using static DroneSimulationBachelor.Implementations.Christofides;

class Program
{
    static Dictionary<string, int> BestSolutionCount = new();

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

        //List<WayPoint> simonExample = new()
        //{
        //    new CentralServer(0, 0),
        //    new DataNode(5,15,"A", TimeSpan.FromSeconds(1)),
        //    new DataNode(4,20,"B", TimeSpan.FromSeconds(1)),
        //    new DataNode(1,19,"C", TimeSpan.FromSeconds(1)),
        //    new DataNode(6,23,"D", TimeSpan.FromSeconds(1)),
        //    new DataNode(2,19,"E", TimeSpan.FromSeconds(1)),
        //    new DataNode(3,14,"F", TimeSpan.FromSeconds(1)),
        //    new DataNode(12,17,"G", TimeSpan.FromSeconds(1)),
        //    new DataNode(4,13,"H", TimeSpan.FromSeconds(1)),
        //};

        //CreateResultDirectory(new Scenario(new List<List<WayPoint>>() { simonExample }, 9), "simon");

        List<List<WayPoint>> randomDataTwelveNodes = new();
        List<List<WayPoint>> randomDataFiftyNodes = new();

        List<List<WayPoint>> outlierDataTwelveNodes = new();
        List<List<WayPoint>> outlierDataFiftyNodes = new();

        List<List<WayPoint>> clusterDataTwelveNodes = new();
        List<List<WayPoint>> clusterDataFiftyNodes = new();

        const int SCENARIO_COUNT = 20;

        for (int creationIndex = 0; creationIndex < SCENARIO_COUNT; creationIndex++)
        {
            randomDataTwelveNodes.Add(GenerateRectangularRandomDataSets(DateTime.MinValue, 12, 1));
            randomDataFiftyNodes.Add(GenerateRectangularRandomDataSets(DateTime.MinValue, 50, 1));

            outlierDataTwelveNodes.Add(GenerateRectangularOutlierDataSets(DateTime.MinValue, 12, 1));
            outlierDataFiftyNodes.Add(GenerateRectangularOutlierDataSets(DateTime.MinValue, 50, 1));

            clusterDataTwelveNodes.Add(GenerateRectangularClusteredDataSets(DateTime.MinValue, 12, 1));
            clusterDataFiftyNodes.Add(GenerateRectangularClusteredDataSets(DateTime.MinValue, 50, 1));
        }

        Scenario random_12 = new(randomDataTwelveNodes, 12);
        Scenario random_50 = new(randomDataFiftyNodes, 50);

        Scenario outlier_12 = new(outlierDataTwelveNodes, 12);
        Scenario outlier_50 = new(outlierDataFiftyNodes, 50);

        Scenario cluster_12 = new(clusterDataTwelveNodes, 12);
        Scenario cluster_50 = new(clusterDataFiftyNodes, 50);

        CreateResultDirectory(random_12, nameof(random_12));
        CreateResultDirectory(random_50, nameof(random_50));
        CreateResultDirectory(outlier_12, nameof(outlier_12));
        CreateResultDirectory(outlier_50, nameof(outlier_50));
        CreateResultDirectory(cluster_12, nameof(cluster_12));
        CreateResultDirectory(cluster_50, nameof(cluster_50));
    }

    private static void CreateResultDirectory(Scenario scenario, string scenarioName)
    {
        foreach (var data in scenario.Data)
        {
            Directory.CreateDirectory($"{scenarioName}");
            JsonHandler.WriteToJson(Path.Combine($"{scenarioName}", "data.json"), scenario);
            GenerateResults(data, Path.Combine($"{scenarioName}", $"{scenario.Data.IndexOf(data)}"));
        }
    }

    private static void GenerateResults(List<WayPoint> data, string dataName)
    {
        //Keep track of all maxReactionTimes
        double[] maxReactionTimes = new double[data.Count-1];

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

        double christofidesMaxReactionTime = CalculateMaxReactionTime(christofidesRoute);
        WriteMetaFile(christofidesRoute, mstPath);

        //Create all routes with enforced Edges and simulate christofides with modified Spanning Trees
        List<List<WayPoint>> modifiedChristofidesRoutes = GenerateModifiedSpanningTreeChristofidesRoute(data);
        int modificationID = 1;

        double minimumReactionTime = christofidesMaxReactionTime;
        foreach (List<WayPoint> route in modifiedChristofidesRoutes)
        {
            double maxReactionTime = CalculateMaxReactionTime(route);
            maxReactionTimes[modificationID-1] = maxReactionTime;
            if(maxReactionTime < minimumReactionTime) minimumReactionTime = maxReactionTime;

            string routeName = $"ChristofidesModifiedOnVertex_{modificationID++}";
            Directory.CreateDirectory(Path.Combine(dataName, routeName));
            WriteMetaFile(route, Path.Combine(dataName, routeName), christofidesRoute);

            ServerResults = SimulateRoute(route, (int)(HamiltonianTourLength(route) * 2));
            WriteCSVandPlot(ServerResults, Path.Combine(dataName, routeName));
        }

        using (StreamWriter file = new(Path.Combine(dataName, "minimumReactionTime.txt")))
        {
            List<double> maxReactionTimesList = maxReactionTimes.ToList();
            int smallestMaxReactionIndex = maxReactionTimesList.IndexOf(maxReactionTimes.Min());
            if(minimumReactionTime == christofidesMaxReactionTime 
                || modifiedChristofidesRoutes[smallestMaxReactionIndex].SequenceEqual(christofidesRoute))
            {
                if (!BestSolutionCount.ContainsKey($"MST_{dataName.Split("\\")[0]}"))
                {
                    BestSolutionCount.Add($"MST_{dataName.Split("\\")[0]}", 1);
                }
                else
                {
                    BestSolutionCount[$"MST_{dataName.Split("\\")[0]}"]++;
                }
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
                if(smallestMaxReactionIndex == modifiedChristofidesRoutes.Count)
                {
                    smallestMRTRoute = modifiedChristofidesRoutes[smallestMaxReactionIndex - 1];
                }
                else
                {
                    smallestMRTRoute = modifiedChristofidesRoutes[smallestMaxReactionIndex];
                }
                WayPoint first = smallestMRTRoute[0];
                WayPoint second = smallestMRTRoute[1];

                bool enforcedEdgeIsLongestEdge = IsMaxDistanceFromStart(smallestMRTRoute, first, second);
                if (enforcedEdgeIsLongestEdge)
                {
                    if (!BestSolutionCount.ContainsKey($"longestEdge_{dataName.Split("\\")[0]}"))
                    {
                        BestSolutionCount.Add($"longestEdge_{dataName.Split("\\")[0]}", 1);
                    }
                    else
                    {
                        BestSolutionCount[$"longestEdge_{dataName.Split("\\")[0]}"]++;
                    }
                }
                else
                {
                    if (!BestSolutionCount.ContainsKey($"other_{dataName.Split("\\")[0]}"))
                    {
                        BestSolutionCount.Add($"other_{dataName.Split("\\")[0]}", 1);
                    }
                    else
                    {
                        BestSolutionCount[$"other_{dataName.Split("\\")[0]}"]++;
                    }
                }

                file.Write($"smallest MRT is the ST with with enforced edge: {first} - {second} and distance {first.DistanceTo(second)}");
                file.WriteLine($"\nTotal length of the tour with smallest MRT is: {HamiltonianTourLength(smallestMRTRoute)}");
                file.WriteLine($"enforced edge == longest edge from start: {enforcedEdgeIsLongestEdge}");
            }
            foreach(var kvp in BestSolutionCount)
            {
                file.WriteLine($"Count of {kvp.Key} as best solution on dataset {dataName.Split("\\")[0]} is: {kvp.Value}");
            }
        }
        HistogramPlotter plotter = new HistogramPlotter();
        plotter.PlotMaxReactionTimesPicture(maxReactionTimes, 5, dataName);
    }

    private static bool IsMaxDistanceFromStart(List<WayPoint> smallestMRTRoute, WayPoint first, WayPoint second)
    {
        double maxDistance = -1;

        foreach(var vertex in smallestMRTRoute)
        {
            if (vertex.Equals(first)) continue;
            maxDistance = first.DistanceTo(vertex) > maxDistance ? first.DistanceTo(vertex) : maxDistance;
        }
        return maxDistance == first.DistanceTo(second);
    }

    private static void WriteMetaFile(List<WayPoint> route, string path, List<WayPoint> mst_route = null)
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
            if (mst_route != null) file.WriteLine($"Is MST: {route.SequenceEqual(mst_route)}");
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
        plotter.PlotMultipleReactionTimeHistogram(paths.ToArray(), directoryPath);
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

    public static List<WayPoint> GenerateRectangularOutlierDataSets(DateTime startTime, int countDataNode, 
        int fixedDataNodePeriod = -1, int xBounds = 100, int yBounds = 100, int outlierCount = 1, int outlierFactor = 1)
    {
        Random random = new Random();
        List<WayPoint> points = new List<WayPoint>();
        int zonesCount = outlierFactor + 1;

        double zoneWidth = (double)xBounds / zonesCount;
        double zoneHeight = (double)yBounds / zonesCount;

        points.Add(new CentralServer(zoneWidth / 2, zoneHeight / 2));

        for (int numberNodesAdded = 0; numberNodesAdded < countDataNode; numberNodesAdded++)
        {
            points.Add(new DataNode
            {
                X = random.NextDouble() * zoneWidth,
                Y = random.NextDouble() * zoneHeight,
                ID = Guid.NewGuid().ToString().Substring(0, 4),
                TransmissionPeriod = TimeSpan.FromSeconds(fixedDataNodePeriod == -1 ? random.Next(0, 4) : fixedDataNodePeriod),
            });
        }

        // Generiere Ausreißer
        for (int i = 0; i < outlierCount; i++)
        {
            points.Add(new DataNode
            {
                X = random.NextDouble() * zoneWidth + (zonesCount-1) * zoneWidth,
                Y = random.NextDouble() * zoneHeight + (zonesCount-1) * zoneHeight,
                ID = Guid.NewGuid().ToString().Substring(0, 4),
                TransmissionPeriod = TimeSpan.FromSeconds(fixedDataNodePeriod == -1 ? random.Next(0, 4) : fixedDataNodePeriod),
            });
        }

        return points;
    }

    public static List<WayPoint> GenerateRectangularClusteredDataSets(DateTime startTime, int countDataNode,
        int fixedDataNodePeriod = -1, int xBounds = 100, int yBounds = 100, int countClusters = 3)
    {
        Random random = new Random();
        List<WayPoint> points = new List<WayPoint>();
        points.Add(new CentralServer(xBounds / 2, yBounds / 2));
        // Berechne die Anzahl der Cluster pro Dimension
        int clustersPerDimension = (int)Math.Ceiling(Math.Sqrt(countClusters));

        // Berechne die Größe jedes Cluster-Quadrats
        double clusterWidth = (double)xBounds / clustersPerDimension;
        double clusterHeight = (double)yBounds / clustersPerDimension;

        // Erstelle eine Liste von verfügbaren Cluster-Positionen
        List<(int x, int y)> availableClusters = new List<(int x, int y)>();
        for (int x = 0; x < clustersPerDimension; x++)
        {
            for (int y = 0; y < clustersPerDimension; y++)
            {
                if (availableClusters.Count < countClusters)
                {
                    availableClusters.Add((x, y));
                }
            }
        }

        List<(int x, int y)> chosenClusters = new();
        for(int i = 0; i < countClusters; i++)
        {
            int chosenIndex = random.Next(0, availableClusters.Count);
            chosenClusters.Add(availableClusters[chosenIndex]);
            availableClusters.RemoveAt(chosenIndex);
        }

        // Verteile die Punkte auf die Cluster
        for (int i = 0; i < countDataNode; i++)
        {
            int clusterIndex = i % countClusters;
            var (clusterX, clusterY) = chosenClusters[clusterIndex];

            // Berechne die Grenzen des gewählten Cluster-Quadrats
            double left = clusterX * clusterWidth;
            double top = clusterY * clusterHeight;

            // Generiere einen zufälligen Punkt innerhalb des Cluster-Quadrats
            double x = left + random.NextDouble() * clusterWidth;
            double y = top + random.NextDouble() * clusterHeight;

            points.Add(new DataNode
            {
                X = x,
                Y = y,
                ID = Guid.NewGuid().ToString().Substring(0, 4),
                TransmissionPeriod = TimeSpan.FromSeconds(fixedDataNodePeriod == -1 ? random.Next(0, 4) : fixedDataNodePeriod)
            });
        }
        return points;
    }
}