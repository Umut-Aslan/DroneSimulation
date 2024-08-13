using DroneSimulationBachelor.Abstractions;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection.Metadata.Ecma335;
using System.Text;
using System.Threading.Tasks;

namespace DroneSimulationBachelor
{
    public class Scenario
    {
        public int Cases { get; set; }
        
        public int NodesPerCase { get; set; }
        public List<List<WayPoint>> Data { get; set; }

        public Scenario(List<List<WayPoint>> data, int nodesPerCase)
        {
            Data = data;
            Cases = Data.Count;
            NodesPerCase = nodesPerCase;
        }

        public Scenario()
        {
            Data = new();
            Cases = 0;
        }
    }
}
