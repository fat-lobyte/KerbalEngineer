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
        private struct BurnSimulationParameters
        {
            // transformed initial angle to vertical
            public double z0;

            // initial velocity
            public double v0;

            // gravitational acceleration
            public double g;

            // thrust to weight ratio
            public double n;

            // Body radius for calculation, Currently unused.
            public double R;
        };


        private struct BurnSimulator
        {
            // horizontal displacement during suicide burn
            public double x { get; private set; }

            // vertical displacement during suicide burn
            public double y { get; private set; }

            // duration of suicide burn
            public double t1 { get; private set; }

            // initial condition parameters
            private BurnSimulationParameters pars;

            // initial condition parameter
            private double C;

            // simulation state variables
            private double psi, z, v, t;
            private double psi_prev, t_prev, v_prev;

            // simulation step variables
            private double z_step;
            private double dt, dx, dy;

            public void init(ref BurnSimulationParameters parameters)
            {
                pars = parameters;

                x = 0;
                y = 0;

                z = pars.z0;

                C = pars.v0 / (Math.Pow(pars.z0, pars.n - 1.0) * (1 + (pars.z0* pars.z0)));
                t1 = C / pars.g * Math.Pow(pars.z0, pars.n - 1.0) * (1.0 / (pars.n - 1.0) + pars.z0 * pars.z0 / (pars.n + 1.0));

                t_prev = t1;
                psi_prev = 2 * Math.Atan(pars.z0);
                v_prev = pars.v0;
            }

            public void simulate(ushort n_steps)
            {
                z_step = pars.z0 / n_steps;

                while (z > 0) step();
            }

            private void step()
            {
                // Sharaf, M.A., & Alaqal, L.A.(2012).Computational Algorithm for Gravity Turn Maneuver, 12 (13).

                psi = 2 * Math.Atan(z);
                v = C * Math.Pow(z, pars.n - 1.0) * (1.0 + z * z);
                t = C / pars.g * Math.Pow(z, pars.n - 1.0) * (1.0 / (pars.n - 1.0) + z * z / (pars.n + 1.0));

                dt = t_prev - t;
                dx = 0.5 * (v * Math.Sin(psi) + v_prev * Math.Sin(psi_prev)) * dt;
                dy = 0.5 * (v * Math.Cos(psi) + v_prev * Math.Cos(psi_prev)) * dt;

                x += dx;
                y += dy;

                t_prev = t;
                psi_prev = psi;
                v_prev = v;

                z -= z_step;
            }
        };

        private static readonly SuicideBurnProcessor s_Instance = new SuicideBurnProcessor();

        private BurnSimulationParameters burn_simulation_parameters = new BurnSimulationParameters();
        private BurnSimulator burn_simulator = new BurnSimulator();
        private double m_RadarAltitude;

        public static double Altitude { get; private set; }

        public static double Distance { get; private set; }

        public static double DeltaV { get; private set; }

        public static double Countdown { get; private set; }

        const ushort N_STEPS = 50;

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
            Orbit orbit = vessel?.orbit;
            CelestialBody body = vessel?.mainBody;

            if (FlightGlobals.currentMainBody == null || vessel == null ||
                SimulationProcessor.LastStage == null || !SimulationProcessor.ShowDetails ||
                FlightGlobals.ship_orbit.PeA >= 0.0 || !Surface.ImpactProcessor.ShowDetails)
            {
                ShowDetails = false;
                return;
            }

            // radius of fictional body which is the radius of the actual body plus altitude at the impact point (including terrain altitude)
            burn_simulation_parameters.R = body.Radius + ImpactProcessor.Altitude;

            // gravity at impact point
            burn_simulation_parameters.g = body.gravParameter /
                (burn_simulation_parameters.R * burn_simulation_parameters.R);

            // TWR at the impact point
            burn_simulation_parameters.n = SimulationProcessor.LastStage.thrust
                / (SimulationProcessor.LastStage.mass * burn_simulation_parameters.g);

            // time of impact
            double impactUT = ImpactProcessor.Time + Planetarium.GetUniversalTime();

            // get orbital velocity and orbital position at time of impact
            Vector3d orbitalVelocityAtImpact, orbitalPosAtImpact;
            orbit.GetOrbitalStateVectorsAtUT(impactUT, out orbitalPosAtImpact, out orbitalVelocityAtImpact);

            // impact velocity accounting for the planet rotation
            Vector3d impactVelocity = orbitalVelocityAtImpact - orbit.GetRotFrameVelAtPos(body, orbitalPosAtImpact);
            burn_simulation_parameters.v0 = impactVelocity.magnitude;

            // impact angle
            double angleToVertical = Vector3d.Angle(-orbitalPosAtImpact, impactVelocity) * Math.PI / 180.0;
            burn_simulation_parameters.z0 = Math.Tan(0.5 * angleToVertical);

            // do suicide burn simulation
            burn_simulator.init(ref burn_simulation_parameters);
            burn_simulator.simulate(N_STEPS);

            m_RadarAltitude = vessel.terrainAltitude > 0.0 ?
                vessel.altitude - vessel.terrainAltitude : vessel.altitude;

            // store results
            Countdown = ImpactProcessor.Time - 0.5 * burn_simulator.t1;
            DeltaV = burn_simulator.t1 / SimulationProcessor.LastStage.time * SimulationProcessor.LastStage.deltaV;
            Altitude = ImpactProcessor.Altitude + burn_simulator.y;
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