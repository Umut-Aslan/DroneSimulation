using System;
using System.Collections.Generic;
using System.IO;
using System.Text.Json;
using System.Text.Json.Serialization;
using DroneSimulationBachelor;
using DroneSimulationBachelor.Abstractions;
using DroneSimulationBachelor.Model;

public class JsonHandler
{
    public static void WriteToJson(string filePath, Scenario scenario)
    {
        var options = new JsonSerializerOptions { WriteIndented = true };
        string jsonString = JsonSerializer.Serialize(scenario, options);
        File.WriteAllText(filePath, jsonString);
    }

    public static Scenario ReadFromJson(string filePath)
    {
        string jsonString = File.ReadAllText(filePath);
        JsonSerializerOptions options = new JsonSerializerOptions { PropertyNameCaseInsensitive = true };
        return JsonSerializer.Deserialize<Scenario>(jsonString, options);
    }
}
