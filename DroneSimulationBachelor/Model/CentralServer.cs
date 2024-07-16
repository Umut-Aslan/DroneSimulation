using System;
using System.Collections.Generic;
using System.Data;
using System.Linq;
using System.Text;
using System.Text.Json.Serialization;
using System.Threading.Tasks;
using DroneSimulationBachelor.Abstractions;

namespace DroneSimulationBachelor.Model
{
    public class CentralServer : WayPoint
    {
        [JsonIgnore]
        public Dictionary<string, Dictionary<TimeSpan, uint>> ReactionTimes;

        [JsonConstructor]
        public CentralServer(double x, double y) : base(x, y, "X")
        {
            ReactionTimes = new();
        }

        public void ReceiveData(string id, List<DateTime> data, DateTime currentTime)
        {
            foreach (DateTime time in data)
            {
                if (!ReactionTimes.ContainsKey(id))
                    ReactionTimes[id] = new();
                TimeSpan reactionTime = currentTime - time;
                if (!ReactionTimes[id].ContainsKey(reactionTime))
                    ReactionTimes[id][reactionTime] = 0;
                ReactionTimes[id][reactionTime]++;
            }
        }
    }
}