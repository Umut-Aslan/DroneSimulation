﻿using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using ScottPlot;
using ScottPlot.MinMaxSearchStrategies;
using ScottPlot.Statistics;

namespace DroneSimulationBachelor
{
    public class HistogramPlotter
    {

        public void PlotReactionTimeHistogram(string path)
        {
            List<double> reactionTimes = ExtractReactionTimes(path);
            GenerateReactionTimePlotPicture(path,reactionTimes);
        }

        private static void GenerateReactionTimePlotPicture(string path, List<double> reactionTimes, int binCount = 50)
        {
            var plt = new Plot();

            double min = reactionTimes.Min();
            double max = reactionTimes.Max();
            
            ScottPlot.Statistics.Histogram hist = new(min, max, binCount);
            hist.AddRange(reactionTimes);

            double barWidth = hist.BinSize * 1.2;
            plt.PlotBar(hist.Bins, hist.Counts, barWidth: barWidth);

            // display vertical lines at points of interest
            var stats = new ScottPlot.Statistics.BasicStats(reactionTimes.ToArray());
            plt.AddVerticalLine(stats.Mean, Color.Black, 2, LineStyle.Solid, $"mean: {stats.Mean:#.}");
            plt.AddVerticalLine(stats.Mean - stats.StDev, Color.Black, 2, LineStyle.Dash, $"1 SD: {stats.StDev:#.}");
            plt.AddVerticalLine(stats.Mean + stats.StDev, Color.Black, 2, LineStyle.Dash);
            plt.AddVerticalLine(stats.Mean - stats.StDev * 2, Color.Black, 2, LineStyle.Dot, $"2 SD: {stats.StDev * 2:#.}");
            plt.AddVerticalLine(stats.Mean + stats.StDev * 2, Color.Black, 2, LineStyle.Dot);
            plt.AddVerticalLine(stats.Min, Color.Gray, 1, LineStyle.Dash, "min/max");
            plt.AddVerticalLine(stats.Max, Color.Gray, 1, LineStyle.Dash);
            plt.Legend(location: Alignment.UpperRight);

            // display histogram probability curve as a line plot
            var funcPlot = plt.AddFunction(hist.GetProbabilityCurve(reactionTimes.ToArray()), Color.DarkOrange, 2, LineStyle.Solid);
            funcPlot.YAxisIndex = 1;
            //plt.AddScatterLines(hist.Bins,hist.Counts, Color.Magenta, 2, LineStyle.Solid);


            plt.Title("Accumulated Distribution");
            plt.YAxis.Label("Count (#)");
            plt.XAxis.Label("Timespan (in sec)");
            plt.YAxis2.Label("distribution");
            plt.YAxis2.Ticks(true);
            plt.SetAxisLimits(xMin: 0, yMin: 0);
            plt.SetAxisLimits(xMin: 0, yMin: 0, yMax: 1.1, yAxisIndex: 1);

            plt.SaveFig($"{path}.png");
        }

        public void PlotMultipleReactionTimeHistogram(string[] paths, string directoryPath)
        {
            List<double> allReactionTimes = new();

            foreach(string path in paths)
            {
                List<double> reactionTimes = ExtractReactionTimes(path);
                //GenerateReactionTimePlotPicture(path, reactionTimes);
                allReactionTimes.AddRange(reactionTimes);
            }
            GenerateReactionTimePlotPicture(Path.Combine(directoryPath, "Accumulated_Distribution"), allReactionTimes, 50);
        }

        public void PlotMaxReactionTimesPicture(double[] maxReactionTimes, int binCount, string path = ".")
        {
            var plt = new Plot();

            double min = maxReactionTimes.Min();
            double max = maxReactionTimes.Max();

            ScottPlot.Statistics.Histogram hist = new(min, max, binCount);
            hist.AddRange(maxReactionTimes);

            double barWidth = hist.BinSize * 1.2;
            plt.PlotBar(hist.Bins, hist.Counts, barWidth: barWidth);

            // display vertical lines at points of interest
            var stats = new ScottPlot.Statistics.BasicStats(maxReactionTimes);
            plt.AddVerticalLine(stats.Mean, Color.Black, 2, LineStyle.Solid, $"mean: {stats.Mean:#.}");
            plt.AddVerticalLine(stats.Mean - stats.StDev, Color.Black, 2, LineStyle.Dash, $"1 SD: {stats.StDev:#.}");
            plt.AddVerticalLine(stats.Mean + stats.StDev, Color.Black, 2, LineStyle.Dash);
            plt.AddVerticalLine(stats.Mean - stats.StDev * 2, Color.Black, 2, LineStyle.Dot, $"2 SD: {stats.StDev * 2:#.}");
            plt.AddVerticalLine(stats.Mean + stats.StDev * 2, Color.Black, 2, LineStyle.Dot);
            plt.AddVerticalLine(stats.Min, Color.Gray, 1, LineStyle.Dash, "min/max");
            plt.AddVerticalLine(stats.Max, Color.Gray, 1, LineStyle.Dash);
            plt.Legend(location: Alignment.UpperRight);

            // display histogram probability curve as a line plot
            var funcPlot = plt.AddFunction(hist.GetProbabilityCurve(maxReactionTimes), Color.DarkOrange, 2, LineStyle.Solid);
            funcPlot.YAxisIndex = 1;
            //plt.AddScatterLines(hist.Bins,hist.Counts, Color.Magenta, 2, LineStyle.Solid);


            plt.Title("Max Reaction Times");
            plt.YAxis.Label("Count (#)");
            plt.XAxis.Label("Max Reaction Time (in sec)");
            plt.YAxis2.Label("distribution");
            plt.YAxis2.Ticks(true);
            plt.SetAxisLimits(yMax: hist.Counts.Max() * 1.1, yAxisIndex: 0);
            plt.SetAxisLimits(yMin: 0, yMax: 1.1, yAxisIndex: 1);

            plt.SaveFig(Path.Combine(path, $"MaximumReactionTimes.png"));
        }

        private static List<double> ExtractReactionTimes(string path)
        {
            List<double> reactionTimes = new();
            // show the histogram counts as a bar plot
            using (StreamReader file = new(path))
            {
                string csvLine = file.ReadLine();
                while (!file.EndOfStream)
                {
                    csvLine = file.ReadLine();
                    string[] tokens = csvLine.Split(';');
                    TimeSpan reactionTime = TimeSpan.FromSeconds(Double.Parse(tokens[0]));
                    int count = int.Parse(tokens[1]);

                    for (int i = 0; i < count; i++)
                    {
                        reactionTimes.Add(reactionTime.TotalSeconds);
                    }
                }
            }
            return reactionTimes;
        }
    }
}
