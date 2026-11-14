using System.Text.Json;
using System.Text.Json.Serialization;

namespace Marps.Engine.Json
{
    /// <summary>JSON options for the contract (camelCase, nulls omitted). The same shapes work for Unity.</summary>
    public static class MarpsJson
    {
        public static readonly JsonSerializerOptions Options = new JsonSerializerOptions
        {
            PropertyNamingPolicy = JsonNamingPolicy.CamelCase,
            DefaultIgnoreCondition = JsonIgnoreCondition.WhenWritingNull,
            PropertyNameCaseInsensitive = true,
        };

        public static string Serialize<T>(T value) => JsonSerializer.Serialize(value, Options);

        public static T Deserialize<T>(string json) => JsonSerializer.Deserialize<T>(json, Options);
    }
}
