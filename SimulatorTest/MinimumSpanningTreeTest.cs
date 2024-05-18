using DroneSimulationBachelor.Abstractions;
using DroneSimulationBachelor.Implementations;
using DroneSimulationBachelor.Model;
using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace SimulatorTest
{
    [TestClass]
    public class MinimumSpanningTreeTest
    {
        [TestMethod]
        public void MinimumSpanningTreeOfFourPointsDoesNotChangeMainGraph()
        {
            DataNode p1 = new DataNode(0, 0, TimeSpan.Zero, DateTime.Now, "A");
            DataNode p2 = new DataNode(0, 5, TimeSpan.Zero, DateTime.Now,"B");
            DataNode p3 = new DataNode(0, 7, TimeSpan.Zero, DateTime.Now,"C");
            DataNode p4 = new DataNode(3, 3, TimeSpan.Zero, DateTime.Now,"F");
            
            List<WayPoint> wayPoints = new List<WayPoint>() { p1, p2, p3, p4 };

            Graph g = new Graph(wayPoints);

            Christofides christ = new Christofides();

            Graph mst = christ.MinimumSpanningTree(g);

            Assert.AreEqual(4, g.Vertices.Count);
            Assert.AreEqual(6, g.Edges.Count);
        }

        [TestMethod]
        public void MinimumSpanningTreeOfFourPointsGivesCorrectNumberOfEdgesAndVertices()
        {
            DataNode p1 = new DataNode(0, 0, TimeSpan.Zero, DateTime.Now, "A");
            DataNode p2 = new DataNode(0, 5, TimeSpan.Zero, DateTime.Now, "B");
            DataNode p3 = new DataNode(0, 7, TimeSpan.Zero, DateTime.Now, "C");
            DataNode p4 = new DataNode(3, 3, TimeSpan.Zero, DateTime.Now, "F");

            List<WayPoint> wayPoints = new List<WayPoint>() { p1, p2, p3, p4 };

            Graph g = new Graph(wayPoints);

            Christofides christ = new Christofides();

            Graph mst = christ.MinimumSpanningTree(g);

            Assert.AreEqual(4, mst.Vertices.Count);
            Assert.AreEqual(3, mst.Edges.Count);
        }

        [TestMethod]
        public void FullGraphOfFourNodesHasSixEdges()
        {
            DataNode p1 = new DataNode(0, 0, TimeSpan.Zero, DateTime.Now, "A");
            DataNode p2 = new DataNode(0, 5, TimeSpan.Zero, DateTime.Now, "B");
            DataNode p3 = new DataNode(0, 7, TimeSpan.Zero, DateTime.Now, "C");
            DataNode p4 = new DataNode(3, 3, TimeSpan.Zero, DateTime.Now, "F");

            List<WayPoint> wayPoints = new List<WayPoint>() { p1, p2, p3, p4 };

            Graph g = new Graph(wayPoints);

            Assert.AreEqual(6, g.Edges.Count);
        }

        [TestMethod]
        public void AddingEdgeTwiceOnlyAddsOne()
        {
            DataNode p1 = new DataNode(0, 0, TimeSpan.Zero, DateTime.Now, "A");
            DataNode p2 = new DataNode(0, 1, TimeSpan.Zero, DateTime.Now, "B");
            DataNode p3 = new DataNode(0, 1, TimeSpan.Zero, DateTime.Now, "C");

            Edge e1 = new(p1,p2);

            Edge e2 = new(p1,p3);

            Graph g = new Graph();
            g.AddEdge(e1);
            g.AddEdge(e2);

            Assert.IsTrue(g.Edges.Count == 1);
        }
    }
}