using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Net;
using KRPC.Client;
using KRPC.Client.Services.KRPC;
using KRPC.Client.Services.SpaceCenter;
using SharpDX;
using SharpDX.Mathematics;

namespace ConsoleApp2
{
    class AscendToOrbitTask : IFlightTask
    {
        public AscendToOrbitTask(VesselController vesselController, VesselDirectionController vesselDirectionController, Forecast forecast, double altitude)
        {
            VesselController = vesselController;
            VesselDirectionController = vesselDirectionController;
            Forecast = forecast;
            Altitude = altitude;
        }

        VesselController VesselController;
        VesselDirectionController VesselDirectionController;
        Forecast Forecast;
        double Altitude;

        Forecast.LandingPrediction landingPrediction;
        double brakeAltitudePrediction;
        bool brakingStarted = false;

        enum Stage
        {
            Ascend,
            WaitForCircularize,
            Circularize
        }

        Stage currentStage = Stage.Ascend;

        static double clamp(double value, double min, double max)
        {
            if (value > max) return max;
            if (value < min) return min;
            return value;
        }
        
        public bool update()
        {
            var orbit = Forecast.predictOrbit();
            var apo = orbit.Apoapsis;
            double velocityNeeded = VesselController.calculateOrbitalVelocityAtAltitude(Altitude);
            double currentVelocity = VesselController.getOrbitalVelocity().Length();
            double shipAcceleration = VesselController.getEnginesAcceleration();
            double timeOfManouver = (velocityNeeded - currentVelocity) / shipAcceleration;
            float mixer = (float)(apo / Altitude);
            var downDirection = VesselController.getGravity();
            downDirection.Normalize();
            var tangential = Vector3.Transform(downDirection, Quaternion.RotationYawPitchRoll(MathUtil.DegreesToRadians(90.0f), 0.0f, 0.0f));
            var direction = -downDirection * (1.0f - mixer) + tangential * (mixer);
            if(mixer > 1.0)
            {
                float newMixer = 1.0f - (mixer - 1.0f);
                direction = downDirection * (1.0f - newMixer) + tangential * (newMixer);
            }
            VesselDirectionController.setTargetDirection(direction);

            if(currentStage == Stage.Ascend)
            {
                Console.WriteLine("[Ascend] Apoapsis {0}", apo);

                if (orbit.Apoapsis < Altitude)
                {
                    VesselController.setThrottle(VesselDirectionController.getOnTargetPercentage());
                }
                else
                {
                    VesselController.setThrottle(0.0f);
                    currentStage = Stage.WaitForCircularize;
                }
            }

            if (currentStage == Stage.WaitForCircularize)
            {
                VesselController.setThrottle(0.0f);
                Console.WriteLine("[Waiting] Time left {0}", orbit.TimeToApoapsis - timeOfManouver * 0.5);
                if (orbit.TimeToApoapsis < timeOfManouver * 0.5)
                {
                    currentStage = Stage.Circularize;
                }
            }
            if (currentStage == Stage.Circularize)
            {
                Console.WriteLine("[Circularize] Apoapsis {0} Periapsis {1} Delta V required {2}", apo, orbit.Periapsis, velocityNeeded - currentVelocity);
                double min = Math.Min(orbit.Periapsis, orbit.Apoapsis);
                if (min < Altitude)
                {
                    VesselController.setThrottle(clamp((Altitude - min) * 0.02, 0.0, 1.0));
                }
                else
                {
                    VesselController.setThrottle(0.0f);
                }
            }

            
            VesselDirectionController.update();
            return false;
        }
    }
}
