// 
//     Kerbal Engineer Redux
// 
//     Copyright (C) 2014 CYBUTEK
// 
//     This program is free software: you can redistribute it and/or modify
//     it under the terms of the GNU General Public License as published by
//     the Free Software Foundation, either version 3 of the License, or
//     (at your option) any later version.
// 
//     This program is distributed in the hope that it will be useful,
//     but WITHOUT ANY WARRANTY; without even the implied warranty of
//     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//     GNU General Public License for more details.
// 
//     You should have received a copy of the GNU General Public License
//     along with this program.  If not, see <http://www.gnu.org/licenses/>.
// 

namespace KerbalEngineer.Flight.Readouts.Vessel
{
    using Extensions;
    using Surface;
    using System;

    public class SuicideBurnProcessor : IUpdatable, IUpdateRequest
    {
        private static readonly SuicideBurnProcessor s_Instance = new SuicideBurnProcessor();
        private double m_Acceleration;
        private double m_Gravity;
        private double m_RadarAltitude;

        public static double Altitude { get; private set; }

        public static double Distance { get; private set; }

        public static double DeltaV { get; private set; }

        public static double Countdown { get; private set; }

        public static SuicideBurnProcessor Instance
        {
            get
            {
                return s_Instance;
            }
        }

        public static bool ShowDetails { get; set; }

        public static double Clamp(double x, double min, double max)
        {
            if (x < min) return min;
            if (x > max) return max;
            return x;
        }


        public void Update()
        {
            var vessel = FlightGlobals.ActiveVessel;
            CelestialBody body = vessel?.mainBody;

            if (FlightGlobals.currentMainBody == null || vessel == null || 
                SimulationProcessor.LastStage == null || !SimulationProcessor.ShowDetails ||
                FlightGlobals.ship_orbit.PeA >= 0.0 || !Surface.ImpactProcessor.ShowDetails )
            {
                ShowDetails = false;
                return;
            }

            Vector3d up = (vessel.CoMD - body.position).normalized;
            double angleFromHorizontal = 90 - Vector3d.Angle(-vessel.srf_velocity, up);
            angleFromHorizontal = Clamp(angleFromHorizontal, 0, 90);

            double sine = Math.Sin(angleFromHorizontal * UtilMath.Deg2Rad);

            m_Gravity = FlightGlobals.currentMainBody.gravParameter / Math.Pow(FlightGlobals.currentMainBody.Radius, 2.0);
            m_Acceleration = SimulationProcessor.LastStage.thrust / SimulationProcessor.LastStage.totalMass;
            m_RadarAltitude = FlightGlobals.ActiveVessel.terrainAltitude > 0.0
                ? FlightGlobals.ship_altitude - FlightGlobals.ActiveVessel.terrainAltitude
                : FlightGlobals.ship_altitude;

            double effectiveDecel = 0.5 * 
                (-2 * m_Gravity * sine 
                    + Math.Sqrt((2 * m_Gravity * sine) * (2 * m_Gravity * sine) 
                        + 4 * (m_Acceleration * m_Acceleration - m_Gravity * m_Gravity)
                    )
                );
            double decelTime = vessel.srfSpeed / effectiveDecel;

            Altitude = Surface.ImpactProcessor.Altitude - vessel.verticalSpeed * 0.5 * decelTime;
            Distance = m_RadarAltitude - Altitude;

            DeltaV = decelTime / SimulationProcessor.LastStage.time * SimulationProcessor.LastStage.deltaV;
            Countdown = Surface.ImpactProcessor.Time - 0.5 * decelTime;

            ShowDetails = !double.IsInfinity(Distance);
        }

        public bool UpdateRequested { get; set; }

        public static void RequestUpdate()
        {
            s_Instance.UpdateRequested = true;
            SimulationProcessor.RequestUpdate();
            ImpactProcessor.RequestUpdate();
        }

        public static void Reset()
        {
            FlightEngineerCore.Instance.AddUpdatable(ImpactProcessor.Instance);
            FlightEngineerCore.Instance.AddUpdatable(SimulationProcessor.Instance);
            FlightEngineerCore.Instance.AddUpdatable(s_Instance);
        }
    }
}