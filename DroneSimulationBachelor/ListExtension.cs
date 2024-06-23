using DroneSimulationBachelor.Abstractions;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace DroneSimulationBachelor
{
    public static class WayPointListExtension
    {
        public static List<WayPoint> Reversed(this List<WayPoint> list)
        {
            List<WayPoint> reversedList = list.Reverse<WayPoint>().ToList();
            return reversedList;
        }
    }
}
