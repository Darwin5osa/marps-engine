using System.Collections.Generic;
using Marps.Engine.Geometry;

namespace Marps.Engine.Tests.Helpers
{
    /// <summary>
    /// Reference legs from reference_routes.json (mock-up planner, A*, 22° limit).
    /// Positions in local meters (x east, z north). Used to validate the engine.
    /// </summary>
    public static class ReferenceData
    {
        public sealed class Leg
        {
            public string Name { get; set; }
            public Point2 From { get; set; }
            public Point2 To { get; set; }
            public double LengthM { get; set; }
            public double Minutes { get; set; }
            public double AscentM { get; set; }
            public double DescentM { get; set; }
        }

        // Base B (300,1700); P1 (750,1700); P2 (1450,1400); P3 (780,800); P4 (130,230)
        public static readonly IReadOnlyList<Leg> Legs = new List<Leg>
        {
            new Leg { Name = "B→P1", From = new Point2(300, 1700), To = new Point2(750, 1700), LengthM = 461, Minutes = 9.1, AscentM = 26, DescentM = 7 },
            new Leg { Name = "P1→P2", From = new Point2(750, 1700), To = new Point2(1450, 1400), LengthM = 787, Minutes = 17.9, AscentM = 69, DescentM = 16 },
            new Leg { Name = "P2→P3", From = new Point2(1450, 1400), To = new Point2(780, 800), LengthM = 934, Minutes = 18.3, AscentM = 15, DescentM = 93 },
            new Leg { Name = "P3→P4", From = new Point2(780, 800), To = new Point2(130, 230), LengthM = 889, Minutes = 15.2, AscentM = 22, DescentM = 22 },
            new Leg { Name = "P4→B", From = new Point2(130, 230), To = new Point2(300, 1700), LengthM = 1529, Minutes = 26.3, AscentM = 39, DescentM = 33 },
        };

        /// <summary>Point on the leg with a 23.7° slope that the reference planner stepped on (X 400, Y 2569133).</summary>
        public static readonly Point2 SteepSpot = new Point2(1300.0, 1333.0);
    }
}
