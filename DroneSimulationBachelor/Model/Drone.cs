using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using DroneSimulationBachelor.Abstractions;

namespace DroneSimulationBachelor.Model
{

    public class Drone
    {
        public DateTime CurrentTime { get; set; }
        SortedDictionary<string, List<DateTime>> NodeData;
        List<WayPoint> Route { get; set; }
        int WayPointIndex;

        public Drone(List<WayPoint> route, DateTime currentTime)
        {
            CurrentTime = currentTime;
            NodeData = new();
            Route = route;
            WayPointIndex = 0;
        }

        public void NextWayPoint()
        {
            WayPoint lastWayPoint = Route[WayPointIndex];
            WayPointIndex = ++WayPointIndex % Route.Count;
            WayPoint currWayPoint = Route[WayPointIndex];
            double distance = currWayPoint.DistanceTo(lastWayPoint);
            //speed = 1 distance unit per minute
            double speed = 1.0;
            CurrentTime = CurrentTime.Add(TimeSpan.FromSeconds(distance * speed));

            if (currWayPoint is CentralServer)
            {
                CentralServer centralServer = (CentralServer)currWayPoint;
                foreach (var kvp in NodeData)
                {
                    centralServer.ReceiveData(kvp.Key, kvp.Value, CurrentTime);
                    NodeData[kvp.Key].Clear();
                }
            }
            else if (currWayPoint is DataNode)
            {
                DataNode dataNode = (DataNode)currWayPoint;
                NodeData[dataNode.ID] = dataNode.GetDataAtTime(CurrentTime);
            }
        }
    }
}
