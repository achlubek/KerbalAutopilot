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
    class AscendHalfVectorTask : IFlightTask
    {
        public AscendHalfVectorTask(VesselController vesselController, VesselDirectionController vesselDirectionController, Forecast forecast, double altitude)
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
            Ascend
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
            var velocityNormalized = VesselController.getVelocity();
            velocityNormalized.Normalize();
            float mixer = (float)(apo / Altitude);
            var downDirection = VesselController.getGravity();
            downDirection.Normalize();
            var tangential = Vector3.Transform(-downDirection, Quaternion.RotationYawPitchRoll(MathUtil.DegreesToRadians(-25.0f), 0.0f, 0.0f));
            var direction = -downDirection * (1.0f - mixer) + tangential * (mixer);
            direction.Normalize();
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
                    return true;
                }
            }
                        
            VesselDirectionController.update();
            return false;
        }
    }
}
