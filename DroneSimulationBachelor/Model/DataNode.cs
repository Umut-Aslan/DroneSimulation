using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Text.Json.Serialization;
using System.Threading.Tasks;
using DroneSimulationBachelor.Abstractions;

namespace DroneSimulationBachelor.Model
{
    public class DataNode : WayPoint
    {
        [JsonIgnore]
        public DateTime LastCollectionTime { get; set; }
        [JsonIgnore]
        public DateTime LastCollectedTimeStamp { get; set; }

        public TimeSpan TransmissionPeriod { get; set; }

        public DataNode(int X, int Y, string ID, TimeSpan TransmissionPeriod) : base(X, Y, ID) 
        {
            TransmissionPeriod = TransmissionPeriod;
        }

        public DataNode()
        {

        }

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
