using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using Marps.Engine.Navigation;
using Xunit;
using static Marps.Engine.Tests.Interface.Fixtures;

namespace Marps.Engine.Tests.Interface
{
    [Trait("Method", "Update")]
    public class UpdateEdgeCaseTests
    {
        [Fact]
        public void NullRequest_Throws()
        {
            Assert.Throws<ArgumentNullException>(() => NewEngine().Update(null));
        }

        [Theory]
        [InlineData(null)]
        [InlineData("")]
        public void MissingSessionId_UsesTheDefaultSession(string sid)
        {
            var engine = NewEngine();
            var req = At(BasePos);
            req.SessionId = sid;
            var first = engine.Update(req);
            Assert.Equal("default", first.SessionId);

            // Both spellings land in the same session: progress made with one is seen with the other.
            var other = At(P1Pos, "default");
            Assert.Equal("P2", engine.Update(other).NextWaypoint.Id);
        }

        [Fact]
        public void SessionIds_AreCaseSensitive()
        {
            var engine = NewEngine();
            ReachP1(engine, "astro");
            Assert.Equal("P2", engine.Update(At(P1Pos, "astro")).NextWaypoint.Id);
            Assert.Equal("B", engine.Update(At(50, 50, "ASTRO")).NextWaypoint.Id);   // a fresh session
        }

        [Theory]
        [InlineData(0)]
        [InlineData(-5.5)]
        [InlineData(1e9)]
        public void MissionTime_IsEchoedAsIs(double t)
        {
            var req = At(BasePos);
            req.MissionTimeS = t;
            Assert.Equal(t, NewEngine().Update(req).MissionTimeS);
        }

        [Theory]
        [InlineData(0)]
        [InlineData(360)]
        [InlineData(720)]
        [InlineData(-90)]
        [InlineData(-450)]
        [InlineData(1e6)]
        public void AnyHeading_GivesARelativeBearingInRange(double heading)
        {
            var resp = NewEngine().Update(At(BasePos.X, BasePos.Z, heading: heading));
            double rel = resp.NextWaypoint.RelativeBearingDeg;
            Assert.InRange(rel, -180.0 + 1e-9, 180.0);
            Assert.Equal(90.0, resp.NextWaypoint.BearingDeg, 6);      // P1 is due east of the base
        }

        [Theory]
        [InlineData(double.NaN, 10)]
        [InlineData(10, double.PositiveInfinity)]
        [InlineData(-1e12, 1e12)]
        public void NonFiniteOrFarPosition_DoesNotThrow_AndIsFlaggedOffMap(double x, double z)
        {
            var resp = NewEngine().Update(At(x, z));
            Assert.False(resp.Position.Inside);
            Assert.Contains(resp.Alerts, a => a.Code == "OFF_MAP");
        }

        [Fact]
        public void ExactlyAtTheReachRadius_CountsAsReached()
        {
            var engine = NewEngine();
            engine.Update(At(BasePos));
            var resp = engine.Update(At(P1Pos.X - 10.0, P1Pos.Z));       // ReachRadiusM = 10
            Assert.Equal("done", resp.Objectives[1].Status);
        }

        [Fact]
        public void JustOutsideTheReachRadius_IsNotReached()
        {
            var engine = NewEngine();
            engine.Update(At(BasePos));
            var resp = engine.Update(At(P1Pos.X - 10.5, P1Pos.Z));
            Assert.Equal("active", resp.Objectives[1].Status);
        }

        [Theory]
        [InlineData("NOPE")]
        [InlineData("p2")]
        [InlineData("A1")]
        public void UnknownOrNonStopActiveWaypoint_IsIgnored(string id)
        {
            var engine = NewEngine();
            engine.Update(At(BasePos));
            var req = At(BasePos);
            req.ActiveWaypointId = id;
            Assert.Equal("P1", engine.Update(req).NextWaypoint.Id);
        }

        [Fact]
        public void ActiveWaypoint_CannotJumpBackwards()
        {
            var engine = NewEngine();
            ReachP1(engine);
            var req = At(P1Pos);
            req.ActiveWaypointId = "P1";                              // already done
            Assert.Equal("P2", engine.Update(req).NextWaypoint.Id);
        }

        [Fact]
        public void ActiveWaypoint_JumpsToTheReturnBase()
        {
            var engine = NewEngine();
            engine.Update(At(BasePos));
            var req = At(50, 10);
            req.ActiveWaypointId = "B";
            var resp = engine.Update(req);
            Assert.Equal("B", resp.NextWaypoint.Id);
            Assert.Equal("active", resp.Objectives[4].Status);         // the return step, not the departure
            Assert.Equal(new[] { "done", "done", "done", "done", "active" }, Statuses(resp));
        }

        [Fact]
        public void ForceReplan_RecalculatesEvenWithTheSameTarget()
        {
            var engine = NewEngine();
            engine.Update(At(BasePos));
            Assert.False(engine.Update(At(BasePos)).Route.Replanned);
            var req = At(BasePos);
            req.ForceReplan = true;
            Assert.True(engine.Update(req).Route.Replanned);
        }

        [Fact]
        public void OffRouteThreshold_ReplansOnlyBeyondIt()
        {
            var engine = NewEngine();
            engine.Update(At(BasePos));                                // route B -> P1 along z = 10
            Assert.False(engine.Update(At(50, 10 + 14)).Route.Replanned);   // 14 m off: OffRouteM = 15
            Assert.True(engine.Update(At(50, 10 + 20)).Route.Replanned);    // 20 m off
        }

        [Fact]
        public void MissionWithoutStops_IsCompleteImmediately()
        {
            var engine = NewEngine(mission: Custom(new[] { Wp("B", "base", 10, 10) }));
            var resp = engine.Update(At(50, 50));
            Assert.True(resp.MissionComplete);
            Assert.Null(resp.Route);
            Assert.Null(resp.NextWaypoint);
            Assert.Empty(resp.Objectives);
            Assert.DoesNotContain(resp.Alerts, a => a.Code == "NO_ROUTE");
        }

        [Fact]
        public void CompletedMission_HasNoRouteAndNoNoRouteAlert()
        {
            var engine = NewEngine();
            var done = CompleteMission(engine);
            Assert.True(done.MissionComplete);
            var later = engine.Update(At(60, 60));
            Assert.True(later.MissionComplete);
            Assert.Null(later.Route);
            Assert.Null(later.NextWaypoint);
            Assert.All(later.Objectives, o => Assert.Equal("done", o.Status));
        }

        [Fact]
        public void UnreachableTarget_RaisesNoRoute()
        {
            var engine = NewEngine(map: Walled(P1Pos));
            var resp = engine.Update(At(BasePos));
            Assert.False(resp.Route.Found);
            Assert.Empty(resp.Route.Points);
            Assert.Contains(resp.Alerts, a => a.Code == "NO_ROUTE" && a.Severity == "danger");
        }

        [Fact]
        public void EmptyStopOrder_ChangesNothing()
        {
            var engine = NewEngine();
            engine.Update(At(BasePos));
            var req = At(BasePos);
            req.StopOrder = new List<string>();
            var resp = engine.Update(req);
            Assert.Equal(new[] { "B", "P1", "P2", "P3", "B" }, Ids(resp));
            Assert.False(resp.Route.Replanned);
            Assert.DoesNotContain(resp.Alerts, a => a.Code == "INVALID_STOP_ORDER");
        }

        [Fact]
        public void EmergencyAndCancelInOneRequest_LeavesEmergencyOn()
        {
            // Documented order: cancel is applied first, then the emergency button.
            var req = At(50, 50);
            req.EmergencyReturn = true;
            req.CancelEmergency = true;
            var resp = NewEngine().Update(req);
            Assert.NotNull(resp.Emergency);
            Assert.Equal("B", resp.NextWaypoint.Id);
        }

        [Fact]
        public void Objectives_AlwaysListEveryStepWithItsIndex()
        {
            var resp = NewEngine().Update(At(BasePos));
            Assert.Equal(5, resp.Objectives.Count);
            Assert.Equal(Enumerable.Range(0, 5), resp.Objectives.Select(o => o.Step));
            Assert.Equal(P2Pos.X, resp.Objectives[2].X);
        }

        [Fact]
        public void ConcurrentCallsOnTheSameSession_NeverThrow_AndStayConsistent()
        {
            var engine = NewEngine();
            var errors = new System.Collections.Concurrent.ConcurrentBag<Exception>();
            Parallel.For(0, 200, new ParallelOptions { MaxDegreeOfParallelism = 8 }, i =>
            {
                try
                {
                    var req = At(10 + (i % 90), 10);
                    if (i % 17 == 0) req.StopOrder = new List<string> { "P3" };
                    if (i % 23 == 0) req.EmergencyReturn = true;
                    if (i % 29 == 0) req.CancelEmergency = true;
                    var resp = engine.Update(req);
                    Assert.Equal(5, resp.Objectives.Count);
                }
                catch (Exception ex) { errors.Add(ex); }
            });
            Assert.Empty(errors);
        }
    }
}
