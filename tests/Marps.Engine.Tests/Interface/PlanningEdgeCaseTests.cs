using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Threading.Tasks;
using Marps.Engine.Geometry;
using Marps.Engine.Navigation;
using Marps.Engine.Routing;
using Marps.Engine.Tests.Helpers;
using Xunit;
using static Marps.Engine.Tests.Interface.Fixtures;

namespace Marps.Engine.Tests.Interface
{
    [Trait("Method", "PlanRoute")]
    public class PlanRouteEdgeCaseTests
    {
        [Fact]
        public void SamePoint_IsAZeroLengthRoute()
        {
            var r = NewEngine().PlanRoute(P1Pos, P1Pos);
            Assert.True(r.Found);
            Assert.Single(r.Points);
            Assert.Equal(0, r.HorizontalLengthM);
            Assert.Equal(0, r.EstimatedMinutes);
        }

        [Fact]
        public void PointsBetweenNodes_SnapToTheNearestNode()
        {
            var r = NewEngine().PlanRoute(new Point2(10.7, 10.3), new Point2(99.2, 10.9));
            Assert.True(r.Found);
            Assert.True(r.Points[0].DistanceTo(new Point2(10.7, 10.3)) <= Cell / Math.Sqrt(2) + 1e-9);
            Assert.True(Last(r.Points).DistanceTo(new Point2(99.2, 10.9)) <= Cell / Math.Sqrt(2) + 1e-9);
        }

        [Fact]
        public void OffMapPoints_AreClampedToTheBorder()
        {
            var r = NewEngine().PlanRoute(new Point2(-500, 10), new Point2(5000, 10));
            Assert.True(r.Found);
            Assert.Equal(0, r.Points[0].X);
            Assert.Equal((Nodes - 1) * Cell, Last(r.Points).X);
        }

        [Theory]
        [InlineData(double.NaN, 10, 100, 10)]
        [InlineData(10, 10, double.PositiveInfinity, 10)]
        [InlineData(10, double.NegativeInfinity, 100, 10)]
        [InlineData(10, 10, 100, double.NaN)]
        public void NonFinitePoints_AreNotFound_InsteadOfAGuess(double fx, double fz, double tx, double tz)
        {
            var r = NewEngine().PlanRoute(new Point2(fx, fz), new Point2(tx, tz));
            Assert.False(r.Found);
            Assert.Empty(r.Points);
        }

        [Fact]
        public void BlockedDestination_IsNotFound()
        {
            Assert.False(NewEngine(map: BlockedNode(P1Pos)).PlanRoute(BasePos, P1Pos).Found);
        }

        [Fact]
        public void BlockedStart_CanStillStepOut()
        {
            // An astronaut standing on a blocked node must still get a way out.
            var r = NewEngine(map: BlockedNode(BasePos)).PlanRoute(BasePos, P1Pos);
            Assert.True(r.Found);
        }

        [Fact]
        public void EnclosedDestination_IsNotFound_Quickly()
        {
            var clock = Stopwatch.StartNew();
            var r = NewEngine(map: Walled(P2Pos)).PlanRoute(BasePos, P2Pos);
            Assert.False(r.Found);
            Assert.True(clock.ElapsedMilliseconds < 2000);
        }

        [Fact]
        public void SteepRampAboveTheLimit_IsNotFound_GentleRampIs()
        {
            var steep = TestTerrain.Build(Nodes, Cell, (x, z) => x > 50 ? (x - 50) * Math.Tan(30 * Math.PI / 180) : 0);
            var gentle = TestTerrain.Build(Nodes, Cell, (x, z) => x > 50 ? (x - 50) * Math.Tan(10 * Math.PI / 180) : 0);
            Assert.False(NewEngine(map: steep).PlanRoute(BasePos, P1Pos).Found);
            var r = NewEngine(map: gentle).PlanRoute(BasePos, P1Pos);
            Assert.True(r.Found);
            Assert.InRange(r.MaxEdgeSlopeDeg, 9.0, 22.0);
            Assert.True(r.AscentM > 0);
        }

        [Fact]
        public void OnFlatGround_ThereAndBackHaveTheSameLength()
        {
            var engine = NewEngine();
            Assert.Equal(engine.PlanRoute(BasePos, P2Pos).HorizontalLengthM, engine.PlanRoute(P2Pos, BasePos).HorizontalLengthM, 6);
        }

        [Fact]
        public void DoesNotTouchSessions()
        {
            var engine = NewEngine();
            engine.Update(At(BasePos));
            engine.PlanRoute(P2Pos, P3Pos);
            Assert.False(engine.Update(At(BasePos)).Route.Replanned);
        }

        [Fact]
        public void ParallelCalls_GiveIdenticalResults()
        {
            var engine = NewEngine();
            var lengths = new double[32];
            Parallel.For(0, lengths.Length, i => lengths[i] = engine.PlanRoute(BasePos, P2Pos).HorizontalLengthM);
            Assert.All(lengths, l => Assert.Equal(lengths[0], l, 9));
        }
    }

    [Trait("Method", "PlanItinerary")]
    public class PlanItineraryEdgeCaseTests
    {
        [Fact]
        public void EmptyList_GivesNoLegs()
        {
            Assert.Empty(NewEngine().PlanItinerary(BasePos, new string[0]));
        }

        [Fact]
        public void NullList_Throws()
        {
            Assert.Throws<ArgumentNullException>(() => NewEngine().PlanItinerary(BasePos, null));
        }

        [Theory]
        [InlineData("NOPE")]
        [InlineData("p1")]
        [InlineData("")]
        [InlineData(null)]
        public void UnknownId_Throws_AndNamesIt(string bad)
        {
            var ex = Assert.Throws<ArgumentException>(() => NewEngine().PlanItinerary(BasePos, new[] { "P1", bad }));
            Assert.Contains("Unknown place", ex.Message);
        }

        [Fact]
        public void AnyPlace_NotOnlyStops_IsAllowed()
        {
            var legs = NewEngine().PlanItinerary(BasePos, new[] { "A1" });
            Assert.True(legs.Single().Route.Found);
        }

        [Fact]
        public void SamePlaceTwice_GivesAZeroLengthLeg()
        {
            var legs = NewEngine().PlanItinerary(BasePos, new[] { "P1", "P1" });
            Assert.Equal(0, legs[1].Route.HorizontalLengthM);
            Assert.True(legs[1].Route.Found);
        }

        [Fact]
        public void UnreachablePlace_IsFlagged_AndTheNextLegStartsThere()
        {
            var legs = NewEngine(map: Walled(P2Pos)).PlanItinerary(BasePos, new[] { "P1", "P2", "P3" });
            Assert.True(legs[0].Route.Found);
            Assert.False(legs[1].Route.Found);
            Assert.Equal(P2Pos, legs[2].From);
            Assert.False(legs[2].Route.Found);                        // it starts inside the wall
        }

        [Fact]
        public void NonFiniteStart_FlagsOnlyTheFirstLeg()
        {
            var legs = NewEngine().PlanItinerary(new Point2(double.NaN, 10), new[] { "P1", "P2" });
            Assert.False(legs[0].Route.Found);
            Assert.True(legs[1].Route.Found);
        }

        [Fact]
        public void LegsChainFromPlaceToPlace()
        {
            var legs = NewEngine().PlanItinerary(new Point2(50, 50), new[] { "P1", "P2", "P3", "B" });
            Assert.Equal(new Point2(50, 50), legs[0].From);
            Assert.Equal(P1Pos, legs[1].From);
            Assert.Equal(P2Pos, legs[2].From);
            Assert.Equal(P3Pos, legs[3].From);
        }
    }

    [Trait("Method", "PlanTrip")]
    public class PlanTripEdgeCaseTests
    {
        private static TripRequest Trip(double x, double z, TripStop dest, params TripStop[] stops) =>
            new TripRequest { X = x, Z = z, Stops = stops.ToList(), Destination = dest };

        [Fact]
        public void NullStopsList_IsADirectTrip()
        {
            var plan = NewEngine().PlanTrip(new TripRequest { X = 10, Z = 10, Stops = null, Destination = TripStop.Place("P1") });
            Assert.Null(plan.Error);
            Assert.Single(plan.Legs);
            Assert.True(plan.Found);
        }

        [Fact]
        public void DestinationAtTheCurrentPoint_IsAZeroLengthTrip()
        {
            var plan = NewEngine().PlanTrip(Trip(10, 10, TripStop.Place("B")));
            Assert.True(plan.Found);
            Assert.Equal(0, plan.LengthM);
            Assert.Single(plan.Legs);
        }

        [Fact]
        public void RepeatedStops_GiveZeroLegs_AndNoDuplicatePointsInTheJoinedRoute()
        {
            var plan = NewEngine().PlanTrip(Trip(10, 10, TripStop.Place("P2"), TripStop.Place("P1"), TripStop.Place("P1")));
            Assert.True(plan.Found);
            Assert.Equal(0, plan.Legs[1].LengthM);
            for (int i = 1; i < plan.Points.Count; i++)
                Assert.True(plan.Points[i].DistanceTo(plan.Points[i - 1]) > 1e-9, $"duplicate point at {i}");
        }

        [Theory]
        [InlineData(-5, 50)]
        [InlineData(50, 500)]
        [InlineData(1e12, 1e12)]
        public void StopOutsideTheMap_IsAnError(double x, double z)
        {
            var plan = NewEngine().PlanTrip(Trip(10, 10, TripStop.Place("B"), TripStop.At(x, z)));
            Assert.False(plan.Found);
            Assert.Contains("Stop 1", plan.Error);
            Assert.Contains("outside the map", plan.Error);
            Assert.Empty(plan.Legs);
        }

        [Fact]
        public void DestinationOutsideTheMap_IsAnError()
        {
            var plan = NewEngine().PlanTrip(Trip(10, 10, TripStop.At(200, 10)));
            Assert.Contains("Destination", plan.Error);
            Assert.Contains("outside the map", plan.Error);
        }

        [Fact]
        public void StartingSlightlyOffTheMap_IsAllowed_LikeNavigation()
        {
            var plan = NewEngine().PlanTrip(Trip(-3, 10, TripStop.Place("P1")));
            Assert.Null(plan.Error);
            Assert.True(plan.Found);
        }

        [Theory]
        [InlineData(double.PositiveInfinity, 10)]
        [InlineData(10, double.NaN)]
        public void NonFiniteStop_IsAnError(double x, double z)
        {
            var plan = NewEngine().PlanTrip(Trip(10, 10, TripStop.Place("B"), TripStop.At(x, z)));
            Assert.Contains("Stop 1", plan.Error);
        }

        [Fact]
        public void EmptyIdWithCoordinates_UsesTheCoordinates()
        {
            var plan = NewEngine().PlanTrip(Trip(10, 10, new TripStop { Id = "", X = 60, Z = 60 }));
            Assert.Null(plan.Error);
            Assert.Equal(60, plan.Legs[0].ToX);
            Assert.Null(plan.Legs[0].ToId);
        }

        [Theory]
        [InlineData(" B")]
        [InlineData("b")]
        public void MalformedId_IsAnUnknownPlace(string id)
        {
            var plan = NewEngine().PlanTrip(Trip(10, 10, TripStop.Place(id)));
            Assert.Contains("unknown place", plan.Error);
        }

        [Fact]
        public void TheFirstBadStop_IsTheOneReported()
        {
            var plan = NewEngine().PlanTrip(Trip(10, 10, TripStop.Place("B"), TripStop.Place("P1"), TripStop.Place("BAD"), TripStop.Place("WORSE")));
            Assert.Contains("Stop 2", plan.Error);
            Assert.Contains("BAD", plan.Error);
        }

        [Fact]
        public void ManyStops_ArePlannedInOrder()
        {
            var stops = Enumerable.Range(0, 25).Select(i => TripStop.At(10 + (i % 5) * 20, 10 + (i / 5) * 20)).ToArray();
            var plan = NewEngine().PlanTrip(Trip(10, 10, TripStop.Place("B"), stops));
            Assert.True(plan.Found);
            Assert.Equal(26, plan.Legs.Count);
            Assert.Equal(Enumerable.Range(0, 26), plan.Legs.Select(l => l.Index));
        }

        [Fact]
        public void PointOverload_NullStops_IsDirect()
        {
            var plan = NewEngine().PlanTrip(BasePos, null, P1Pos);
            Assert.True(plan.Found);
            Assert.Single(plan.Legs);
        }

        [Fact]
        public void PointOverload_NonFiniteDestination_IsAnError()
        {
            var plan = NewEngine().PlanTrip(BasePos, new Point2[0], new Point2(double.NaN, 0));
            Assert.NotNull(plan.Error);
        }

        [Fact]
        public void ErrorsStillReportComputeTime()
        {
            var plan = NewEngine().PlanTrip(new TripRequest { X = 10, Z = 10 });
            Assert.True(plan.ComputeMs >= 0);
        }

        [Fact]
        public void RunsSafelyAlongsideNavigation()
        {
            var engine = NewEngine();
            var errors = new System.Collections.Concurrent.ConcurrentBag<Exception>();
            Parallel.For(0, 40, i =>
            {
                try
                {
                    if (i % 2 == 0) engine.Update(At(10 + i, 10, "s" + (i % 4)));
                    else Assert.True(engine.PlanTrip(Trip(10, 10, TripStop.Place("B"), TripStop.Place("P2"))).Found);
                }
                catch (Exception ex) { errors.Add(ex); }
            });
            Assert.Empty(errors);
        }
    }
}
