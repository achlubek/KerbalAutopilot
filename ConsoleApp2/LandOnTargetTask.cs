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
    class LandOnTargetTask : IFlightTask
    {
        public LandOnTargetTask(VesselController vesselController, VesselDirectionController vesselDirectionController, Forecast forecast, Vector3 target)
        {
            VesselController = vesselController;
            VesselDirectionController = vesselDirectionController;
            Forecast = forecast;
            Target = target;
            landingSpeedPID = new PercentageDerivativeController(0.2, 1.5, 0.0);
        }

        VesselController VesselController;
        VesselDirectionController VesselDirectionController;
        Forecast Forecast;
        Vector3 Target;
        PercentageDerivativeController landingSpeedPID;

        Forecast.LandingPrediction landingPrediction;
        double brakeAltitudePrediction;
        bool brakingStarted = false;

        enum Stage
        {
            HighAltitudeCorrections,
            BallisticDescend,
            ReentryBurn,
            BallisticDescend2,
            LandingBurn,
            Landed
        }

        Stage currentStage = Stage.HighAltitudeCorrections;

        static double clamp(double value, double min, double max)
        {
            if (value > max) return max;
            if (value < min) return min;
            return value;
        }

        private void updateHighAltitudeCorrections()
        {
            var targetRelative = Target - landingPrediction.Position;
            var targetRelativeNormalized = targetRelative;
            targetRelativeNormalized.Normalize();

            var desiredDirection = targetRelativeNormalized;// * Math.Min(1.0f, targetRelative.Length() * 0.01f);
            desiredDirection.Normalize();
            VesselDirectionController.setTargetDirection(desiredDirection);
            var vesselVelocity = VesselController.getVelocity();
            float minimalCorrectiveThrottle = (float)VesselDirectionController.getOnTargetPercentage() *
                Math.Min(1.0f, targetRelative.Length() * 0.0001f) * 1.0f;// * Math.Min(1.0f, vesselVelocity.Length() * 0.005f);
            VesselController.setThrottle(minimalCorrectiveThrottle);
            Console.WriteLine("[High altitude pinpoint] Prediction mismatch {0}", targetRelative.Length());
            /*
            var vesselVelocity = VesselController.getVelocity();
            var vesselVelocityNormalized = vesselVelocity;
            vesselVelocityNormalized.Normalize();
            VesselDirectionController.setTargetDirection(-vesselVelocityNormalized);*/
        }

        private Vector3 getCorrectionVector()
        {
            var vesselPosition = VesselController.getPosition();
            var vesselVelocity = VesselController.getVelocity();
            var vesselVelocityNormalized = vesselVelocity;
            vesselVelocityNormalized.Normalize();
            var gravityAtTarget = VesselController.getGravityAtPoint(Target);
            gravityAtTarget.Normalize();
            var newPos = Target - gravityAtTarget * (float)VesselController.getAltitude();
            var targetRelative = newPos - VesselController.getPosition();
            var targetRelativeNormalized = targetRelative;
            targetRelativeNormalized.Normalize();

            var correctionVector = targetRelativeNormalized - vesselVelocityNormalized;
            correctionVector.Normalize();
            return correctionVector;
        }

        private void updateBallisticDescend()
        {
            var vesselVelocity = VesselController.getVelocity();
            var vesselVelocityNormalized = vesselVelocity;
            vesselVelocityNormalized.Normalize();
            var gravityAtTarget = VesselController.getGravityAtPoint(Target);
            gravityAtTarget.Normalize();
            var newPos = Target - gravityAtTarget * (float)VesselController.getAltitude();
            var targetRelative = newPos - VesselController.getPosition();
            var targetRelativeNormalized = targetRelative;
            targetRelativeNormalized.Normalize();
            var desiredDirection = -vesselVelocityNormalized - targetRelativeNormalized * 0.88f * Math.Min(1.0f, targetRelative.Length() * 0.0165f);
            desiredDirection.Normalize();
            VesselDirectionController.setTargetDirection(desiredDirection);
            VesselController.setThrottle(0.0f);

            Console.WriteLine("[First Ballistic flight] Prediction mismatch {0}", targetRelative.Length());
        }

        private void updateBallisticDescend2()
        {
            var vesselVelocity = VesselController.getVelocity();
            var vesselVelocityNormalized = vesselVelocity;
            vesselVelocityNormalized.Normalize();
            var targetRelative = Target - landingPrediction.Position;
            var targetRelativeNormalized = targetRelative;
            targetRelativeNormalized.Normalize();
            // Agressive!
            var desiredDirection = -vesselVelocityNormalized - targetRelativeNormalized * 0.88f * Math.Min(1.0f, targetRelative.Length() * 0.0065f);
            desiredDirection.Normalize();
            VesselDirectionController.setTargetDirection(desiredDirection);
            VesselController.setThrottle(0.0f);

            Console.WriteLine("[Second Ballistic flight] Prediction mismatch {0}", targetRelative.Length());
        }

        private void setAcceleration(double value)
        {
            double percentage = 1.0;
            while (Math.Abs(VesselController.getEnginesAccelerationPrediction(percentage) - value) > 0.1)
            {
                percentage -= (VesselController.getEnginesAccelerationPrediction(percentage) - value) * 0.01f;
                percentage = clamp(percentage, 0.0f, 1.0f);
                //Console.WriteLine("TWR: {0}, Percentage {1}", calculateAcceleration(), percentage);
            }
            VesselController.setThrustPercentage(percentage);
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

        private void updateReentryBurn()
        {
            var vesselVelocity = VesselController.getVelocity();
            var vesselVelocityNormalized = vesselVelocity;
            vesselVelocityNormalized.Normalize();
            var gravityAtTarget = VesselController.getGravityAtPoint(Target);
            gravityAtTarget.Normalize();
            var newPos = Target - gravityAtTarget * (float)VesselController.getAltitude();
            var targetRelative = newPos - VesselController.getPosition();
            var targetRelativeNormalized = targetRelative;
            targetRelativeNormalized.Normalize();
            var desiredDirection = -vesselVelocityNormalized + targetRelativeNormalized * 0.54f * Math.Min(1.0f, targetRelative.Length() * 0.003f);
            desiredDirection.Normalize();
            VesselDirectionController.setTargetDirection(desiredDirection);
            VesselController.setThrottle(1.0f);
            
            Console.WriteLine("[Reentry burn] Prediction mismatch {0}", targetRelative.Length());
        }

        private void updateLandingBurn()
        {
            var targetRelative = Target - landingPrediction.Position;
            var targetRelativeNormalized = targetRelative;
            targetRelativeNormalized.Normalize();

            var vesselVelocity = VesselController.getVelocity();
            var vesselVelocityNormalized = vesselVelocity;
            vesselVelocityNormalized.Normalize();

            double velocityValue = vesselVelocity.Length();
            float mixer = (float)clamp((velocityValue - 150.0) * 0.05, -1.0, 1.0);

            //var desiredDirection = -vesselVelocityNormalized - mixer * targetRelativeNormalized * 0.78f * Math.Min(1.0f, targetRelative.Length() * 0.0065f);
            var desiredDirection = -vesselVelocityNormalized + targetRelativeNormalized * 0.8f * Math.Min(1.0f, targetRelative.Length() * 0.03f);
            desiredDirection.Normalize();
            VesselDirectionController.setTargetDirection(desiredDirection);

            if (brakeAltitudePrediction <= 0.0) // VesselController.getAltitude() * 0.2f + 4.0f)
            {
                //VesselController.setThrottle(clamp(1.0f * vesselVelocity.Length() * 0.01f, 0.0, 1.0));
                //VesselController.setThrottle(1.0f);
                brakingStarted = true;
            }
            else
            {
                //VesselController.setThrottle(VesselController.getThrottle() * 0.99f);
            }
            if (brakingStarted)
            {
                double hoverPercentage = predictThrustPercentageForAcceleration(VesselController.getGravity().Length());
                double val = vesselVelocity.Length() - (5.0f + VesselController.getAltitude() * 0.09f);// - landingSpeedPID.Calculate(17.0f, vesselVelocity.Length());
                if (brakeAltitudePrediction <= 0.0) val = 1.0;
               // if (vesselVelocity.Length() > VesselController.getAltitude() * 0.2f + 5.0) val = 1.0;
                VesselController.setThrottle(clamp(val, 0.0, 1.0));
            }
            if(vesselVelocity.Length() < 60.0)
            {
                //setAcceleration(VesselController.getGravity().Length() * 2.0);
                //    VesselController.setThrottle(0.0f);
            }

            if (VesselController.getAltitude() < 500)
            {
                VesselController.setLandingGearState(true);
            }

            Console.WriteLine("[Landing burn] Prediction mismatch {0} Brake prediction {1}", targetRelative.Length(), brakeAltitudePrediction);
        }


        private void updateLanded()
        {
            var vesselVelocity = VesselController.getVelocity();
            var vesselVelocityNormalized = vesselVelocity;
            vesselVelocityNormalized.Normalize();
            var targetRelative = Target - landingPrediction.Position;
            var targetRelativeNormalized = targetRelative;
            targetRelativeNormalized.Normalize();
            var desiredDirection = -vesselVelocityNormalized;
            desiredDirection.Normalize();
            VesselDirectionController.setTargetDirection(desiredDirection);
            VesselController.setThrottle(0.0f);

            Console.WriteLine("[Landed] Prediction mismatch {0}", targetRelative.Length());
        }

        public bool update()
        {
            landingPrediction = Forecast.predictLandPosition();
            brakeAltitudePrediction = Forecast.predictImmediateRetrogradeBurnStopAltitude();

            var altitude = VesselController.getAltitude();
            var velocity = VesselController.getVelocity().Length();
            var targetRelative = Target - landingPrediction.Position;
            double missDistance = targetRelative.Length();

            if (currentStage == Stage.HighAltitudeCorrections && missDistance < 500)
            {
                currentStage = Stage.BallisticDescend;
            }
            if (currentStage == Stage.BallisticDescend && altitude < 20000)
            {
                currentStage = Stage.ReentryBurn;
            }
            if (currentStage == Stage.ReentryBurn && velocity < 300)
            {
                currentStage = Stage.BallisticDescend2;
            }
            if (currentStage == Stage.BallisticDescend2 && brakeAltitudePrediction <= 100.0)
            {
                currentStage = Stage.LandingBurn;
            }
            if (currentStage == Stage.LandingBurn && altitude <= 20.0 && velocity < 2.0f)
            {
                currentStage = Stage.Landed;
            }

            if (currentStage == Stage.HighAltitudeCorrections) updateHighAltitudeCorrections();
            if (currentStage == Stage.BallisticDescend) updateBallisticDescend();
            if (currentStage == Stage.BallisticDescend2) updateBallisticDescend2();
            if (currentStage == Stage.ReentryBurn) updateReentryBurn();
            if (currentStage == Stage.LandingBurn) updateLandingBurn();
            if (currentStage == Stage.Landed) updateLanded();

            VesselDirectionController.update();


            return false;
        }
    }
}
