using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace DroneSimulationBachelor.Abstractions
{
    public interface IRouteGenerator
    {
        List<WayPoint> GenerateRoute(List<WayPoint> dataPoints);
    }

}
