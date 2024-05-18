using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using ScottPlot;
using ScottPlot.MinMaxSearchStrategies;

namespace DroneSimulationBachelor
{
    public class HistogramPlotter
    {

        public void PlotHistogram(string path)
        {
            var plt = new Plot();

            List<double> reactionTimes = new();
            // show the histogram counts as a bar plot
            using (StreamReader file = new(path))
            {
                string csvLine = file.ReadLine();
                while (!file.EndOfStream)
                {
                    csvLine = file.ReadLine();
                    string[] tokens = csvLine.Split(';');
                    TimeSpan reactionTime = TimeSpan.Parse(tokens[0]);
                    int count = int.Parse(tokens[1]);

                    for(int i = 0; i < count; i++)
                    {
                        reactionTimes.Add(reactionTime.TotalSeconds);
                    }
                }
            }
            double min = reactionTimes.Min();
            double max = reactionTimes.Max();
            ScottPlot.Statistics.Histogram hist = new(min, max, 50);
            hist.AddRange(reactionTimes);

            // customize the plot style
            double barWidth = hist.BinSize * 1.2;

            plt.PlotBar(hist.Bins, hist.Counts, barWidth: barWidth);
            plt.Title(path);
            plt.YAxis.Label("Count (#)");
            plt.XAxis.Label("Timespan (in sec)");
            plt.SetAxisLimits(yMin: 0);

            plt.SaveFig($"{path}.png");
        }

    }
}
