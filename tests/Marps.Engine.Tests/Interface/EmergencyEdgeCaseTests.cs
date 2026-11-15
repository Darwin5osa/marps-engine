using System;
using System.Linq;
using Marps.Engine.Geometry;
using Marps.Engine.Navigation;
using Xunit;
using static Marps.Engine.Tests.Interface.Fixtures;

namespace Marps.Engine.Tests.Interface
{
    [Trait("Method", "StartEmergencyReturn")]
    public class StartEmergencyReturnEdgeCaseTests
    {
        private static NavigationRequest Timed(Point2 p, double t)
        {
            var req = At(p);
            req.MissionTimeS = t;
            return req;
        }

        [Fact]
        public void NullRequest_Throws()
        {
            Assert.Throws<ArgumentNullException>(() => NewEngine().StartEmergencyReturn(null));
        }

        [Fact]
        public void PressingTwice_KeepsTheFirstActivationAndDoesNotReplan()
        {
            var engine = NewEngine();
            var first = engine.StartEmergencyReturn(Timed(new Point2(60, 30), 100));
            Assert.True(first.Route.Replanned);
            var second = engine.StartEmergencyReturn(Timed(new Point2(60, 30), 200));
            Assert.Equal(100, second.Emergency.ActivatedAtS);
            Assert.False(second.Route.Replanned);
        }

        [Fact]
        public void PressedAtTheBase_IsArrivedAtOnce()
        {
            var resp = NewEngine().StartEmergencyReturn(At(BasePos));
            Assert.True(resp.Emergency.Arrived);
            Assert.Contains(resp.Alerts, a => a.Code == "EMERGENCY_RETURN" && a.Severity == "info");
            Assert.DoesNotContain(resp.Alerts, a => a.Code == "EMERGENCY_RETURN" && a.Severity == "danger");
        }

        [Fact]
        public void ArrivalBoundary_IsTheReachRadius()
        {
            var engine = NewEngine();
            Assert.True(engine.StartEmergencyReturn(At(BasePos.X + 10, BasePos.Z)).Emergency.Arrived);
            Assert.False(engine.Update(At(BasePos.X + 10.5, BasePos.Z)).Emergency.Arrived);
        }

        [Fact]
        public void AfterTheMissionIsComplete_StillGuidesToTheBase()
        {
            var engine = NewEngine();
            CompleteMission(engine);
            var resp = engine.StartEmergencyReturn(At(60, 60));
            Assert.True(resp.Emergency.Active);
            Assert.False(resp.MissionComplete);
            Assert.Equal("B", resp.Route.Target);
            Assert.True(resp.Route.Found);
            Assert.All(resp.Objectives, o => Assert.Equal("done", o.Status));   // nothing pending to resume
        }

        [Fact]
        public void UnreachableBase_ReportsNoRoute_WithoutADistance()
        {
            var engine = NewEngine(map: Walled(BasePos, inner: 4, outer: 6));
            var resp = engine.StartEmergencyReturn(At(60, 60));
            Assert.False(resp.Route.Found);
            var alert = resp.Alerts.First();
            Assert.Equal("EMERGENCY_RETURN", alert.Code);
            Assert.DoesNotContain(" m,", alert.Message);
            Assert.Contains(resp.Alerts, a => a.Code == "NO_ROUTE");
        }

        [Fact]
        public void OffMap_ReportsBothAlerts()
        {
            var resp = NewEngine().StartEmergencyReturn(At(-50, -50));
            Assert.Contains(resp.Alerts, a => a.Code == "EMERGENCY_RETURN");
            Assert.Contains(resp.Alerts, a => a.Code == "OFF_MAP");
        }

        [Fact]
        public void MissionWithoutBase_RefusesEveryTime()
        {
            var engine = NewEngine(mission: Custom(new[] { Wp("P1", "science", 60, 60) }, "P1"));
            for (int i = 0; i < 2; i++)
            {
                var resp = engine.StartEmergencyReturn(At(10, 10));
                Assert.Null(resp.Emergency);
                Assert.Contains(resp.Alerts, a => a.Code == "NO_BASE");
                Assert.Equal("P1", resp.NextWaypoint.Id);
            }
        }

        [Fact]
        public void BaseOnlyAtTheEnd_IsUsed()
        {
            var places = new[] { Wp("P1", "science", 60, 60), Wp("HOME", "base", 20, 20) };
            var engine = NewEngine(mission: Custom(places, "P1", "HOME"));
            Assert.Equal("HOME", engine.Base.Id);
            var resp = engine.StartEmergencyReturn(At(100, 100));
            Assert.Equal("HOME", resp.NextWaypoint.Id);
            Assert.Equal(new[] { "pending", "active" }, Statuses(resp));
        }

        [Fact]
        public void StandingOnTheActiveStopWhenPressed_DoesNotCompleteIt()
        {
            var engine = NewEngine();
            ReachP1(engine);
            engine.Update(At(100, 60));
            var resp = engine.StartEmergencyReturn(At(P2Pos));
            Assert.Equal("pending", resp.Objectives[2].Status);
        }

        [Fact]
        public void EmergencyRoute_EndsAtTheBase()
        {
            var resp = NewEngine().StartEmergencyReturn(At(100, 100));
            Assert.True(Last(resp.Route.Points).DistanceTo(BasePos) < 3.0);
            Assert.True(resp.Route.Points[0].DistanceTo(new Point2(100, 100)) < 0.01);   // starts at the astronaut
        }
    }

    [Trait("Method", "CancelEmergencyReturn")]
    public class CancelEmergencyReturnEdgeCaseTests
    {
        [Fact]
        public void NullRequest_Throws()
        {
            Assert.Throws<ArgumentNullException>(() => NewEngine().CancelEmergencyReturn(null));
        }

        [Fact]
        public void WhenNotInEmergency_IsANoOp()
        {
            var engine = NewEngine();
            engine.Update(At(BasePos));
            var resp = engine.CancelEmergencyReturn(At(BasePos));
            Assert.Null(resp.Emergency);
            Assert.Equal("P1", resp.Route.Target);
            Assert.False(resp.Route.Replanned);
            Assert.DoesNotContain(resp.Alerts, a => a.Code == "EMERGENCY_RETURN");
        }

        [Fact]
        public void CancellingTwice_IsHarmless()
        {
            var engine = NewEngine();
            ReachP1(engine);
            engine.StartEmergencyReturn(At(100, 50));
            engine.CancelEmergencyReturn(At(100, 50));
            var resp = engine.CancelEmergencyReturn(At(100, 50));
            Assert.Null(resp.Emergency);
            Assert.Equal("P2", resp.NextWaypoint.Id);
        }

        [Fact]
        public void AfterCancel_StopsAreCreditedAgain()
        {
            var engine = NewEngine();
            ReachP1(engine);
            engine.StartEmergencyReturn(At(100, 50));
            engine.CancelEmergencyReturn(At(100, 50));
            var resp = engine.Update(At(P2Pos));
            Assert.Equal("done", resp.Objectives[2].Status);
            Assert.Equal("P3", resp.NextWaypoint.Id);
        }

        [Fact]
        public void CancelWhileStandingOnAStop_CreditsIt()
        {
            var engine = NewEngine();
            ReachP1(engine);
            engine.StartEmergencyReturn(At(100, 50));
            engine.Update(At(P2Pos));                                  // not credited during the emergency
            var resp = engine.CancelEmergencyReturn(At(P2Pos));
            Assert.Equal("done", resp.Objectives[2].Status);
        }

        [Fact]
        public void CancelAfterACompletedMission_IsCompleteAgain()
        {
            var engine = NewEngine();
            CompleteMission(engine);
            engine.StartEmergencyReturn(At(60, 60));
            var resp = engine.CancelEmergencyReturn(At(60, 60));
            Assert.True(resp.MissionComplete);
            Assert.Null(resp.Route);
        }

        [Fact]
        public void PressingAgainAfterCancel_StartsANewEmergency()
        {
            var engine = NewEngine();
            var first = At(60, 60);
            first.MissionTimeS = 10;
            engine.StartEmergencyReturn(first);
            engine.CancelEmergencyReturn(At(60, 60));
            var again = At(60, 60);
            again.MissionTimeS = 99;
            var resp = engine.StartEmergencyReturn(again);
            Assert.Equal(99, resp.Emergency.ActivatedAtS);
            Assert.True(resp.Route.Replanned);
        }
    }
}
