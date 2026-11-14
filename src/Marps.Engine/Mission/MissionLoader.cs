using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text.Json;
using System.Text.Json.Serialization;
using Marps.Engine.Json;
using Marps.Engine.Terrain;

namespace Marps.Engine.Mission
{
    /// <summary>Reads mission.json and hazards.json and converts them to local meters.</summary>
    public static class MissionLoader
    {
        public static MissionDefinition Load(string missionJsonPath, string hazardJsonPath, TerrainMetadata meta)
        {
            var file = JsonSerializer.Deserialize<MissionFileDto>(File.ReadAllText(missionJsonPath), MarpsJson.Options)
                       ?? throw new InvalidDataException("mission.json is empty.");

            var waypoints = (file.Places ?? new List<PlaceDto>()).Select(p =>
            {
                var local = meta.ProjectionToLocal(p.X, p.Y);
                return new Waypoint
                {
                    Id = p.Id,
                    Type = p.Type,
                    Name = p.Name,
                    Reason = p.Reason,
                    Position = local,
                    ProjectionX = p.X,
                    ProjectionY = p.Y,
                };
            });

            var zones = string.IsNullOrEmpty(hazardJsonPath) || !File.Exists(hazardJsonPath)
                ? new List<HazardZone>()
                : LoadZones(hazardJsonPath, meta);

            return new MissionDefinition(file.Name, file.Objective, waypoints, file.Order ?? new string[0], zones);
        }

        public static IReadOnlyList<HazardZone> LoadZones(string hazardJsonPath, TerrainMetadata meta)
        {
            var dtos = JsonSerializer.Deserialize<List<HazardZoneDto>>(File.ReadAllText(hazardJsonPath), MarpsJson.Options)
                       ?? new List<HazardZoneDto>();
            return dtos.Select(d =>
            {
                // box = [ULX, ULY, LRX, LRY] in projection
                double ulx = d.Box[0], uly = d.Box[1], lrx = d.Box[2], lry = d.Box[3];
                var min = meta.ProjectionToLocal(ulx, lry);
                var max = meta.ProjectionToLocal(lrx, uly);
                var center = meta.ProjectionToLocal(d.Center[0], d.Center[1]);
                return new HazardZone
                {
                    Type = d.Type,
                    AreaM2 = d.AreaM2,
                    Center = center,
                    BoxMin = min,
                    BoxMax = max,
                    MaxSlopeDeg = d.MaxSlopeDeg,
                    FractionOver15 = d.FractionOver15,
                };
            }).ToList();
        }

        private sealed class MissionFileDto
        {
            [JsonPropertyName("name")] public string Name { get; set; }
            [JsonPropertyName("objective")] public string Objective { get; set; }
            [JsonPropertyName("order")] public string[] Order { get; set; }
            [JsonPropertyName("places")] public List<PlaceDto> Places { get; set; }
        }

        private sealed class PlaceDto
        {
            [JsonPropertyName("id")] public string Id { get; set; }
            [JsonPropertyName("type")] public string Type { get; set; }
            [JsonPropertyName("name")] public string Name { get; set; }
            [JsonPropertyName("X")] public double X { get; set; }
            [JsonPropertyName("Y")] public double Y { get; set; }
            [JsonPropertyName("reason")] public string Reason { get; set; }
        }

        private sealed class HazardZoneDto
        {
            [JsonPropertyName("type")] public string Type { get; set; }
            [JsonPropertyName("area_m2")] public double AreaM2 { get; set; }
            [JsonPropertyName("center")] public double[] Center { get; set; }
            [JsonPropertyName("box_ULX_ULY_LRX_LRY")] public double[] Box { get; set; }
            [JsonPropertyName("max_slope_deg")] public double MaxSlopeDeg { get; set; }
            [JsonPropertyName("fraction_over_15")] public double FractionOver15 { get; set; }
        }
    }
}
