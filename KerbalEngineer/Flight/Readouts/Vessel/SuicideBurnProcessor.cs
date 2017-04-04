// 
//     Kerbal Engineer Redux
// 
//     Copyright (C) 2014 CYBUTEK, 2017 fat-lobyte
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
    using Surface;
    using System;

    public class SuicideBurnProcessor : IUpdatable, IUpdateRequest
    {
        private static readonly SuicideBurnProcessor s_Instance = new SuicideBurnProcessor();
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

        public void Update()
        {
            var vessel = FlightGlobals.ActiveVessel;
            CelestialBody body = vessel?.mainBody;

            if (FlightGlobals.currentMainBody == null || vessel == null ||
                SimulationProcessor.LastStage == null || !SimulationProcessor.ShowDetails ||
                FlightGlobals.ship_orbit.PeA >= 0.0 || !Surface.ImpactProcessor.ShowDetails)
            {
                ShowDetails = false;
                return;
            }

            // get time of impact
            double impactUT = ImpactProcessor.Time + Planetarium.GetUniversalTime();

            // get orbital velocity and orbital position at time of impact
            Vector3d orbitalVelocityAtImpact, orbitalPosAtImpact;
            vessel.orbit.GetOrbitalStateVectorsAtUT(impactUT, out orbitalPosAtImpact, out orbitalVelocityAtImpact);

            // calculate impact angle and impact velocity considering the planet rotation
            Vector3d velocityAtImpact = orbitalVelocityAtImpact - vessel.orbit.GetRotFrameVelAtPos(body, orbitalPosAtImpact);
            double angleToVertical = Vector3d.Angle(-orbitalPosAtImpact, velocityAtImpact) * Math.PI / 180.0;
            double speedAtImpact = velocityAtImpact.magnitude;


            // gravity at impact
            m_Gravity = FlightGlobals.currentMainBody.gravParameter /
                           Math.Pow(FlightGlobals.currentMainBody.Radius + ImpactProcessor.Altitude, 2.0);

            m_RadarAltitude = FlightGlobals.ActiveVessel.terrainAltitude > 0.0
                ? FlightGlobals.ship_altitude - FlightGlobals.ActiveVessel.terrainAltitude
                : FlightGlobals.ship_altitude;


            // Sharaf, M.A., & Alaqal, L.A.(2012).Computational Algorithm for Gravity Turn Maneuver, 12 (13).

            // calculate TWR at the impact point
            double n = SimulationProcessor.LastStage.thrust / (SimulationProcessor.LastStage.mass * m_Gravity);
                
               // SimulationProcessor.LastStage.thrustToWeight;
            double z0 = Math.Tan(0.5 * angleToVertical);
            double C = speedAtImpact / (Math.Pow(z0, n - 1.0) * (1 + (z0*z0)));

            double decelTime = C / m_Gravity * Math.Pow(z0, n - 1.0) * (1.0 / (n - 1.0) + z0 * z0 / (n + 1.0));

            // we simulate by varying the z parameter, and we split it up in N_STEPS.
            uint N_STEPS = 100;
            double z_step = z0 / N_STEPS;
            double psi_step = angleToVertical / N_STEPS;

            // simulation variables
            double psi, z, v, t, dt;
            double x = 0.0, y = 0.0;


            double t_prev = decelTime;
            double psi_prev = angleToVertical;
            double v_prev = speedAtImpact;

            // suicide burn simulation loop
            //for (double z = z0; z >= 0.0; z -= z_step)
            for (psi = angleToVertical; psi >= 0.0; psi -= psi_step)
            {
                //psi = 2 * Math.Atan(z);
                z = Math.Tan(0.5 * psi);
                v = C * Math.Pow(z, n - 1.0) * (1.0 + z * z);
                t = C / m_Gravity * Math.Pow(z, n - 1.0) * (1.0/(n - 1.0) + z*z / (n + 1.0));

                dt = t_prev - t;
                x += 0.5 * (v * Math.Sin(psi) + v_prev * Math.Sin(psi_prev)) * dt;
                y += 0.5 * (v * Math.Cos(psi) + v_prev * Math.Cos(psi_prev)) * dt; 

                t_prev = t;
                psi_prev = psi;
                v_prev = v;
            }

            // store results
            Countdown = ImpactProcessor.Time - 0.5 * decelTime;
            DeltaV = decelTime / SimulationProcessor.LastStage.time * SimulationProcessor.LastStage.deltaV;
            Altitude = ImpactProcessor.Altitude + y;
            Distance = m_RadarAltitude - Altitude;

            ShowDetails = !double.IsInfinity(Distance);
        }
        
        public bool UpdateRequested { get; set; }

        public static void RequestUpdate()
        {
            s_Instance.UpdateRequested = true;
            SimulationProcessor.RequestUpdate();
        }

        public static void Reset()
        {
            FlightEngineerCore.Instance.AddUpdatable(ImpactProcessor.Instance);
            FlightEngineerCore.Instance.AddUpdatable(SimulationProcessor.Instance);
            FlightEngineerCore.Instance.AddUpdatable(s_Instance);
        }
    }
}