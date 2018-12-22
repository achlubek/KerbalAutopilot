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
    class LandTask : IFlightTask
    {
        public LandTask(VesselController vesselController, VesselDirectionController vesselDirectionController, Forecast forecast)
        {
            VesselController = vesselController;
            VesselDirectionController = vesselDirectionController;
            Forecast = forecast;
            landingSpeedPID = new PercentageDerivativeController(0.2, 1.5, 0.0);
        }

        VesselController VesselController;
        VesselDirectionController VesselDirectionController;
        Forecast Forecast;
        PercentageDerivativeController landingSpeedPID;

        Forecast.LandingPrediction landingPrediction;
        double brakeAltitudePrediction;

        enum Stage
        {
            Landing,
            Landed
        }

        Stage currentStage = Stage.Landing;

        static double clamp(double value, double min, double max)
        {
            if (value > max) return max;
            if (value < min) return min;
            return value;
        }

        private double predictThrustPercentageForAcceleration(double value)
        {
            double percentage = 1.0;
            while (Math.Abs(VesselController.getEnginesAccelerationPrediction(percentage) - value) > 0.1)
            {
                percentage -= (VesselController.getEnginesAccelerationPrediction(percentage) - value) * 0.01f;
                percentage = clamp(percentage, 0.0f, 1.0f);
                //Console.WriteLine("TWR: {0}, Percentage {1}", calculateAcceleration(), percentage);
            }
            return percentage;
        }
        
        private void updateLandingBurn()
        {
            var vesselVelocity = VesselController.getVelocity();
            var vesselVelocityNormalized = vesselVelocity;
            vesselVelocityNormalized.Normalize();

            VesselDirectionController.setTargetDirection(-vesselVelocityNormalized);
            
            double hoverPercentage = predictThrustPercentageForAcceleration(VesselController.getGravity().Length());
            double val = vesselVelocity.Length() - (5.0f + VesselController.getAltitude() * 0.09f);// - landingSpeedPID.Calculate(17.0f, vesselVelocity.Length());
            if (brakeAltitudePrediction <= 0.0) val = 1.0;
            // if (vesselVelocity.Length() > VesselController.getAltitude() * 0.2f + 5.0) val = 1.0;
            VesselController.setThrottle(clamp(val, 0.0, 1.0));
           
            if (VesselController.getAltitude() < 500)
            {
                VesselController.setLandingGearState(true);
            }

            Console.WriteLine("[Landing burn] Brake prediction {0}", brakeAltitudePrediction);
        }


        private void updateLanded()
        {
            var vesselVelocity = VesselController.getVelocity();
            var vesselVelocityNormalized = vesselVelocity;
            vesselVelocityNormalized.Normalize();

            VesselDirectionController.setTargetDirection(-vesselVelocityNormalized);
            VesselController.setThrottle(0.0f);

            Console.WriteLine("[Landed]");
        }

        public bool update()
        {
            landingPrediction = Forecast.predictLandPosition();
            brakeAltitudePrediction = Forecast.predictImmediateRetrogradeBurnStopAltitude();

            var altitude = VesselController.getAltitude();
            var velocity = VesselController.getVelocity().Length();

            if (currentStage == Stage.Landing && altitude <= 20.0 && velocity < 2.0f)
            {
                currentStage = Stage.Landed;
            }

            if (currentStage == Stage.Landing) updateLandingBurn();
            if (currentStage == Stage.Landed) updateLanded();

            VesselDirectionController.update();
            
            return false;
        }
    }
}
