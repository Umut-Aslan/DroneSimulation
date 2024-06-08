using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using DroneSimulationBachelor.Abstractions;

namespace DroneSimulationBachelor.Model
{
    public class DataNode : WayPoint
    {
        DateTime LastCollectionTime;
        DateTime LastCollectedTimeStamp;
        TimeSpan TransmissionPeriod;

        public DataNode(int x, int y, TimeSpan period, DateTime lastCollectionTime, string id) : base(x, y, id)
        {
            TransmissionPeriod = period;
            LastCollectionTime = lastCollectionTime;
        }

        public List<DateTime> GetDataAtTime(DateTime currentTime)
        {
            List<DateTime> data = new();
            DateTime firstNewTimeStamp = LastCollectedTimeStamp + TransmissionPeriod;
            for (DateTime time = firstNewTimeStamp; time <= currentTime; time += TransmissionPeriod)
            {
                data.Add(time);
                LastCollectedTimeStamp = time;
            }
            LastCollectionTime = currentTime;
            return data;
        }
    }
}
