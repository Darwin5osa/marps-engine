using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text.Json;
using System.Threading;
using System.Threading.Tasks;
using Marps.Engine.Abstractions;
using Marps.Engine.Geometry;
using Marps.Engine.Mission;
using Marps.Engine.Navigation;
using Marps.Engine.Terrain;
using Marps.Engine.Tests.Helpers;
using Xunit;

namespace Marps.Engine.Tests
{
    /// <summary>Behaviour when the interfaces around the engine fail: sensor, weather, HUD sink, data files.</summary>
    public class FaultToleranceTests
    {
        private static MissionDefinition OneTarget() => new MissionDefinition(
            "faults", "faults",
            new[] { new Waypoint { Id = "P1", Type = "science", Name = "P1", Position = new Point2(60, 60) } },
            new[] { "P1" },
            null);

        private static NavigationRequest Walker(double x = 10, double z = 10) =>
            new NavigationRequest { SessionId = "f", X = x, Z = z, HeadingDeg = 45 };

        private sealed class FakeSensor : IAstronautSensor
        {
            public Func<NavigationRequest> Next = () => Walker();
            public NavigationRequest Read() => Next();
        }

        private sealed class FakeWeather : ISpaceWeatherProvider
        {
            public bool Fail;
            public SpaceWeather Current() => Fail ? throw new InvalidOperationException("weather down") : null;
        }

        private sealed class FakeSink : IHudSink
        {
            public bool Fail;
            public int Published;
            public void Publish(NavigationResponse response)
            {
                if (Fail) throw new InvalidOperationException("display disconnected");
                Published++;
            }
        }

        private static NavigationService Service(FakeSensor s, FakeSink sink = null, FakeWeather w = null, NavigationEngine engine = null) =>
            new NavigationService(
                engine ?? new NavigationEngine(TestTerrain.Flat(41, 2.0), OneTarget()),
                s, sink ?? new FakeSink(), w);

        [Fact]
        public void Sensor_Throws_StepReportsFault_AndRecovers()
        {
            var sensor = new FakeSensor { Next = () => throw new IOException("cable unplugged") };
            var service = Service(sensor);

            var faulty = service.Step();
            Assert.Contains(faulty.Alerts, a => a.Code == "SENSOR_FAULT");
            Assert.Equal(1, service.SensorFaults);
            Assert.Equal("cable unplugged", service.LastError);

            sensor.Next = () => Walker();
            var recovered = service.Step();
            Assert.DoesNotContain(recovered.Alerts, a => a.Code == "SENSOR_FAULT");
            Assert.NotNull(recovered.Route);
            Assert.Equal(1, service.SensorFaults);
        }

        [Theory]
        [InlineData(double.NaN, 10, 0)]
        [InlineData(10, double.PositiveInfinity, 0)]
        [InlineData(10, 10, double.NaN)]
        public void Sensor_InvalidReading_IsRejectedBeforeTheEngine(double x, double z, double heading)
        {
            var sensor = new FakeSensor { Next = () => new NavigationRequest { X = x, Z = z, HeadingDeg = heading } };
            var service = Service(sensor);
            var response = service.Step();
            Assert.Contains(response.Alerts, a => a.Code == "SENSOR_FAULT");
            Assert.Null(response.Route);
            Assert.Equal(1, service.SensorFaults);
        }

        [Fact]
        public void Sensor_ReturnsNull_IsTreatedAsFault()
        {
            var service = Service(new FakeSensor { Next = () => null });
            Assert.Contains(service.Step().Alerts, a => a.Code == "SENSOR_FAULT");
        }

        [Fact]
        public void Weather_Throws_NavigationContinues_WithWarning()
        {
            var weather = new FakeWeather { Fail = true };
            var service = Service(new FakeSensor(), w: weather);
            var response = service.Step();

            Assert.Contains(response.Alerts, a => a.Code == "WEATHER_UNAVAILABLE");
            Assert.NotNull(response.Route);
            Assert.True(response.Route.Found);
            Assert.Equal(1, service.WeatherFaults);
        }

        [Fact]
        public void Sink_Throws_EngineStateIsKept()
        {
            var sink = new FakeSink { Fail = true };
            var service = Service(new FakeSensor(), sink);

            var first = service.Step();
            Assert.NotNull(first.Route);
            Assert.Equal(1, service.SinkFaults);

            sink.Fail = false;
            var second = service.Step();
            Assert.Equal(1, sink.Published);
            Assert.False(second.Route.Replanned, "the route cached before the sink failed should be reused");
        }

        [Fact]
        public void TerrainWithNaNHeight_IsRejected()
        {
            var meta = new TerrainMetadata { Width = 3, Height = 3, OriginX = 0, OriginY = 0, SideM = 4 };
            var heights = new float[9];
            heights[4] = float.NaN;
            var ex = Assert.Throws<ArgumentException>(() => new TerrainMap(meta, heights, new byte[9]));
            Assert.Contains("not a finite", ex.Message);
        }

        [Fact]
        public void TerrainWithUnknownHazardCode_IsRejected()
        {
            var meta = new TerrainMetadata { Width = 3, Height = 3, OriginX = 0, OriginY = 0, SideM = 4 };
            var hazard = new byte[9];
            hazard[0] = 9;
            var ex = Assert.Throws<ArgumentException>(() => new TerrainMap(meta, new float[9], hazard));
            Assert.Contains("0..3", ex.Message);
        }

        [Fact]
        public void MalformedMissionJson_FailsWithAClearError()
        {
            string path = Path.Combine(Path.GetTempPath(), "marps-bad-" + Guid.NewGuid().ToString("N") + ".json");
            File.WriteAllText(path, "{ \"name\": \"x\", \"places\": [ { \"id\": ");
            var meta = new TerrainMetadata { Width = 3, Height = 3, OriginX = 0, OriginY = 0, SideM = 4 };
            Assert.ThrowsAny<JsonException>(() => MissionLoader.Load(path, null, meta));
        }

        [Fact]
        public void MissionReferencingUnknownPlace_IsRejected()
        {
            var ex = Assert.Throws<ArgumentException>(() => new MissionDefinition(
                "m", "m", new[] { new Waypoint { Id = "A", Type = "base", Position = new Point2(0, 0) } },
                new[] { "A", "MISSING" }, null));
            Assert.Contains("MISSING", ex.Message);
        }

        [Fact]
        public void Engine_UnderConcurrentInvalidAndExtremeInputs_NeverThrowsOrHangs()
        {
            TerrainMap map = TestTerrain.Flat(41, 2.0);
            using var engine = new NavigationEngine(map, OneTarget(), new NavigationOptions { BackgroundReplanning = true, MaxSessions = 8 });
            var exceptions = new System.Collections.Concurrent.ConcurrentBag<Exception>();
            var values = new double[] { double.NaN, double.PositiveInfinity, double.NegativeInfinity, 1e12, -1e12, 0, 10, 60, 79, double.Epsilon };

            var task = Task.Run(() => Parallel.For(0, 4, new ParallelOptions { MaxDegreeOfParallelism = 4 }, t =>
            {
                var rnd = new Random(t);
                for (int i = 0; i < 400; i++)
                {
                    try
                    {
                        engine.Update(new NavigationRequest
                        {
                            SessionId = "s" + rnd.Next(20),
                            X = values[rnd.Next(values.Length)],
                            Z = values[rnd.Next(values.Length)],
                            HeadingDeg = values[rnd.Next(values.Length)],
                            ActiveWaypointId = rnd.Next(3) == 0 ? "ZZZ" : null,
                            ForceReplan = rnd.Next(5) == 0,
                        });
                    }
                    catch (Exception ex)
                    {
                        exceptions.Add(ex);
                    }
                }
            }));

            Assert.True(task.Wait(TimeSpan.FromSeconds(60)), "engine calls did not finish in time");
            Assert.True(exceptions.IsEmpty, string.Join("; ", exceptions.Select(e => e.GetType().Name + ": " + e.Message).Distinct()));
        }
    }
}
