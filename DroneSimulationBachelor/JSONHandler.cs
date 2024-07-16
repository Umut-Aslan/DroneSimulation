using System;
using System.Collections.Generic;
using System.IO;
using System.Text.Json;
using System.Text.Json.Serialization;
using DroneSimulationBachelor.Abstractions;
using DroneSimulationBachelor.Model;

public class JsonHandler
{
    public static void WriteToJson(string filePath, List<WayPoint> points)
    {
        var options = new JsonSerializerOptions { WriteIndented = true };
        string jsonString = JsonSerializer.Serialize(points, options);
        File.WriteAllText(filePath, jsonString);
    }

    public static List<WayPoint> ReadFromJson(string filePath)
    {
        string jsonString = File.ReadAllText(filePath);
        JsonSerializerOptions options = new JsonSerializerOptions { PropertyNameCaseInsensitive = true };
        return JsonSerializer.Deserialize<List<WayPoint>>(jsonString, options);
    }
}
