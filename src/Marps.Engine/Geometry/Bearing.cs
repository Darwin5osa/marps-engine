using System;

namespace Marps.Engine.Geometry
{
    /// <summary>Rumbos en grados: 0 = norte (+Z), 90 = este (+X), sentido horario.</summary>
    public static class Bearing
    {
        /// <summary>Rumbo absoluto desde <paramref name="from"/> hacia <paramref name="to"/>, en [0, 360).</summary>
        public static double Between(Point2 from, Point2 to)
        {
            double dx = to.X - from.X;
            double dz = to.Z - from.Z;
            return Normalize360(Math.Atan2(dx, dz) * 180.0 / Math.PI);
        }

        /// <summary>Bearing relative to the look direction, in (-180, 180]. Negative = left.</summary>
        public static double Relative(double bearingDeg, double headingDeg)
        {
            double d = Normalize360(bearingDeg) - Normalize360(headingDeg); // (-360, 360)
            if (d > 180.0) d -= 360.0;
            else if (d <= -180.0) d += 360.0;
            return d;
        }

        public static double Normalize360(double deg)
        {
            double r = deg % 360.0;
            if (r < 0) r += 360.0;
            return r;
        }
    }
}
