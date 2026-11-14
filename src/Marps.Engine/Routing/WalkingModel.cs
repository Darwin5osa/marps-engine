namespace Marps.Engine.Routing
{
    /// <summary>
    /// Walking time in suit, in minutes, per step:
    ///   t = FlatMinPerM * distancia_horizontal + ClimbMinPerM * subida + DescentMinPerM * bajada
    /// Coefficients fitted by least squares to the 5 reference legs (reference_routes.json, with the 22° rule):
    /// maximum error 0.11 min. This is a fit to 5 data points (3 parameters): it must be validated with real data.
    /// </summary>
    public sealed class WalkingModel
    {
        public static readonly WalkingModel Fitted = new WalkingModel(0.013937, 0.090788, 0.042108);

        public WalkingModel(double flatMinPerM, double climbMinPerM, double descentMinPerM)
        {
            FlatMinPerM = flatMinPerM;
            ClimbMinPerM = climbMinPerM;
            DescentMinPerM = descentMinPerM;
        }

        public double FlatMinPerM { get; }
        public double ClimbMinPerM { get; }
        public double DescentMinPerM { get; }

        /// <summary>Cost (minutes) of a step. <paramref name="dh"/> = destination height minus origin height.</summary>
        public double EdgeMinutes(double horizontalM, double dh)
        {
            return dh >= 0
                ? FlatMinPerM * horizontalM + ClimbMinPerM * dh
                : FlatMinPerM * horizontalM - DescentMinPerM * dh;
        }

        /// <summary>Admissible lower bound for A*: never overestimates.</summary>
        public double LowerBoundMinutes(double euclideanM) => FlatMinPerM * euclideanM;
    }
}
