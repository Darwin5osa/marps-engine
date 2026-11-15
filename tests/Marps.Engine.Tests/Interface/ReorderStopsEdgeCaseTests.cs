using System;
using System.Collections.Generic;
using System.Linq;
using Xunit;
using static Marps.Engine.Tests.Interface.Fixtures;

namespace Marps.Engine.Tests.Interface
{
    [Trait("Method", "ReorderStops")]
    public class ReorderStopsEdgeCaseTests
    {
        [Fact]
        public void NullRequest_Throws()
        {
            Assert.Throws<ArgumentNullException>(() => NewEngine().ReorderStops(null, new[] { "P1" }));
        }

        [Fact]
        public void NullOrder_Throws()
        {
            Assert.Throws<ArgumentNullException>(() => NewEngine().ReorderStops(At(BasePos), null));
        }

        [Fact]
        public void EmptyOrder_ChangesNothing()
        {
            var engine = NewEngine();
            engine.Update(At(BasePos));
            var resp = engine.ReorderStops(At(BasePos), new string[0]);
            Assert.Equal(new[] { "B", "P1", "P2", "P3", "B" }, Ids(resp));
            Assert.False(resp.Route.Replanned);
            Assert.DoesNotContain(resp.Alerts, a => a.Code == "INVALID_STOP_ORDER");
        }

        [Fact]
        public void TheCurrentOrder_DoesNotReplan()
        {
            var engine = NewEngine();
            engine.Update(At(BasePos));
            var resp = engine.ReorderStops(At(BasePos), new[] { "P1", "P2", "P3" });
            Assert.False(resp.Route.Replanned);
            Assert.Equal("P1", resp.Route.Target);
        }

        [Fact]
        public void KeepingTheActiveStopFirst_ReordersTheRestWithoutReplanning()
        {
            var engine = NewEngine();
            engine.Update(At(BasePos));
            var resp = engine.ReorderStops(At(BasePos), new[] { "P1", "P3", "P2" });
            Assert.Equal(new[] { "B", "P1", "P3", "P2", "B" }, Ids(resp));
            Assert.False(resp.Route.Replanned);                       // the leg to P1 is still valid
        }

        [Theory]
        [InlineData("p3")]          // ids are case-sensitive
        [InlineData(" P3")]         // no trimming
        [InlineData("P3 ")]
        [InlineData(null)]
        public void MalformedIds_AreRejected_AndTheRouteIsKept(string bad)
        {
            var engine = NewEngine();
            engine.Update(At(BasePos));
            var resp = engine.ReorderStops(At(BasePos), new[] { "P2", bad });
            Assert.Contains(resp.Alerts, a => a.Code == "INVALID_STOP_ORDER");
            Assert.Equal(new[] { "B", "P1", "P2", "P3", "B" }, Ids(resp));
            Assert.Equal("P1", resp.Route.Target);
            Assert.False(resp.Route.Replanned);
        }

        [Fact]
        public void RejectionMessage_NamesTheOffendingId()
        {
            var resp = NewEngine().ReorderStops(At(BasePos), new[] { "P2", "XYZ" });
            Assert.Contains("XYZ", resp.Alerts.Single(a => a.Code == "INVALID_STOP_ORDER").Message);
        }

        [Fact]
        public void MissionWithoutBases_LetsTheFirstStopMoveToo()
        {
            var mission = Custom(new[] { Wp("P1", "science", 100, 10), Wp("P2", "science", 100, 100), Wp("P3", "science", 10, 100) },
                "P1", "P2", "P3");
            var resp = NewEngine(mission: mission).ReorderStops(At(BasePos), new[] { "P3" });
            Assert.Equal(new[] { "P3", "P1", "P2" }, Ids(resp));
            Assert.Equal("P3", resp.NextWaypoint.Id);
        }

        [Fact]
        public void RepeatedStopInTheMission_MovesOnlyTheFirstPendingOccurrence()
        {
            var places = new[] { Wp("B", "base", 10, 10), Wp("P1", "science", 100, 10), Wp("P2", "science", 100, 100) };
            var engine = NewEngine(mission: Custom(places, "B", "P1", "P2", "P1", "B"));
            engine.Update(At(BasePos));
            var resp = engine.ReorderStops(At(BasePos), new[] { "P2" });
            Assert.Equal(new[] { "B", "P2", "P1", "P1", "B" }, Ids(resp));
        }

        [Fact]
        public void AfterTheMissionIsComplete_IsAcceptedAndChangesNothing()
        {
            var engine = NewEngine();
            CompleteMission(engine);
            var resp = engine.ReorderStops(At(BasePos), new[] { "P3", "P2", "P1" });
            Assert.True(resp.MissionComplete);
            Assert.DoesNotContain(resp.Alerts, a => a.Code == "INVALID_STOP_ORDER");
            Assert.Equal(new[] { "B", "P1", "P2", "P3", "B" }, Ids(resp));
        }

        [Fact]
        public void OnlyOneStopLeft_IsANoOp()
        {
            var engine = NewEngine();
            ReachP1(engine);
            engine.Update(At(P2Pos));                                  // only P3 (and the return) left
            var resp = engine.ReorderStops(At(P2Pos), new[] { "P3" });
            Assert.Equal("P3", resp.NextWaypoint.Id);
            Assert.DoesNotContain(resp.Alerts, a => a.Code == "INVALID_STOP_ORDER");
        }

        [Fact]
        public void FirstCallOfANewSession_CanReorder()
        {
            var resp = NewEngine().ReorderStops(At(BasePos, "fresh"), new[] { "P3" });
            Assert.Equal(new[] { "B", "P3", "P1", "P2", "B" }, Ids(resp));
            Assert.Equal("P3", resp.NextWaypoint.Id);
        }

        [Fact]
        public void ChangingTheCallersListAfterwards_HasNoEffect()
        {
            var engine = NewEngine();
            var order = new List<string> { "P3", "P2", "P1" };
            engine.ReorderStops(At(BasePos), order);
            order.Clear();
            order.Add("P2");
            Assert.Equal(new[] { "B", "P3", "P2", "P1", "B" }, Ids(engine.Update(At(BasePos))));
        }

        [Fact]
        public void EveryIdOfTheMission_IncludingBasesAndDoneStops_IsAccepted()
        {
            var engine = NewEngine();
            ReachP1(engine);
            var resp = engine.ReorderStops(At(P1Pos), new[] { "B", "P1", "P3", "P2" });
            Assert.DoesNotContain(resp.Alerts, a => a.Code == "INVALID_STOP_ORDER");
            Assert.Equal(new[] { "B", "P1", "P3", "P2", "B" }, Ids(resp));
            Assert.Equal("P3", resp.NextWaypoint.Id);
        }

        [Fact]
        public void ReorderedRoute_IsToTheNewStop()
        {
            var resp = NewEngine().ReorderStops(At(BasePos), new[] { "P2" });
            Assert.True(resp.Route.Found);
            Assert.True(Last(resp.Route.Points).DistanceTo(P2Pos) < 3.0);
        }

        [Fact]
        public void ReorderInOneSession_LeavesOthersAlone()
        {
            var engine = NewEngine();
            engine.ReorderStops(At(BasePos, "a"), new[] { "P3" });
            Assert.Equal("P1", engine.Update(At(BasePos, "b")).NextWaypoint.Id);
        }
    }
}
