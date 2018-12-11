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
            landingSpeedPID = new PercentageDerivativeController(1.0, 0.0, 0.0);
            correctionsPID = new PercentageDerivativeControllerVec3(10.6f, 2.6f, 0.0f);
        }

        VesselController VesselController;
        VesselDirectionController VesselDirectionController;
        Forecast Forecast;
        Vector3 Target;
        PercentageDerivativeController landingSpeedPID;

        Forecast.LandingPrediction landingPrediction;
        Forecast.LandingPrediction brakingLandingPrediction;

        PercentageDerivativeControllerVec3 correctionsPID;

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

        static Vector3 clamp(Vector3 value, double min, double max)
        {
            return new Vector3(
                    (float)clamp(value.X, min, max),
                    (float)clamp(value.Y, min, max),
                    (float)clamp(value.Z, min, max)
                );
        }

        private void updateHighAltitudeCorrections()
        {
            var targetRelative = Target - landingPrediction.Position;
            var targetRelativeNormalized = targetRelative;
            targetRelativeNormalized.Normalize();

            var overShootTarget = Target + targetRelativeNormalized * 1000.0f;
            targetRelativeNormalized = overShootTarget - landingPrediction.Position;
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
        private void updateBallisticDescend()
        {
            var vesselVelocity = VesselController.getVelocity();
            var vesselVelocityNormalized = vesselVelocity;
            vesselVelocityNormalized.Normalize();
            var targetRelative = Target - landingPrediction.Position;
            var targetRelativeNormalized = targetRelative;
            targetRelativeNormalized.Normalize();

            var desiredDirection = -vesselVelocityNormalized;// - targetRelativeNormalized * 0.58f * Math.Min(1.0f, targetRelative.Length() * 0.015f);

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

            var desiredDirection = -vesselVelocityNormalized - targetRelativeNormalized * 0.88f * Math.Min(1.0f, targetRelative.Length() * 0.015f);

            desiredDirection.Normalize();
            VesselDirectionController.setTargetDirection(desiredDirection);
            VesselController.setThrottle(0.0f);

            Console.WriteLine("[Second Ballistic flight] Prediction mismatch {0}, Brake prediction {1}", targetRelative.Length(), brakeAltitudePrediction);
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
            var targetRelative = Target - landingPrediction.Position;
            var targetRelativeNormalized = targetRelative;
            targetRelativeNormalized.Normalize();
            var correction = clamp(correctionsPID.Calculate(Vector3.Zero, targetRelative * 0.010f), -1.0, 1.0);

            var desiredDirection = -vesselVelocityNormalized - correction * 0.3f;
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

            var downDirection = VesselController.getGravity();
            downDirection.Normalize();
            var vesselVelocity = VesselController.getVelocity();
            var vesselVelocityNormalized = vesselVelocity;
            vesselVelocityNormalized.Normalize();


            var altitude = VesselController.getAltitude();
            var targetAltitude = VesselController.getAltitudeAtPoint(Target);
            altitude -= targetAltitude;

            float tiltMultiplier = 0.38f;
            float relativeMultiplier = 0.0125f;
            if (altitude < 7000)
            {
                tiltMultiplier = 0.18f;
                relativeMultiplier = 0.0225f;
            }
            if (altitude < 5000)
            {
                tiltMultiplier = 0.08f;
                relativeMultiplier = 0.0825f;
            }
            if (altitude < 2500)
            {
                tiltMultiplier = 0.0f;
            }

            var desiredDirection = -vesselVelocityNormalized + targetRelativeNormalized * tiltMultiplier * Math.Min(1.0f, targetRelative.Length() * relativeMultiplier);
            if (vesselVelocity.Length() < 15.0)
            {
                bool upsidedown = Vector3.Dot(downDirection, vesselVelocityNormalized) < 0.0;
                desiredDirection = -(upsidedown ? downDirection : vesselVelocityNormalized);// + targetRelativeNormalized * 0.10f * Math.Min(1.0f, targetRelative.Length() * 0.0225f);
            }

            VesselDirectionController.setTargetDirection(desiredDirection);
            
            var velocity = Vector3.Dot(vesselVelocity, -downDirection);
            
            var pidCalculatedThrottle = landingSpeedPID.Calculate(-altitude * 0.06f - 5.0f, velocity);
            //Console.WriteLine("PID Result : {0}", pidCalculatedThrottle);

            double hoverPercentage = predictThrustPercentageForAcceleration(VesselController.getGravity().Length()) * 2.0 - 1.0;

            pidCalculatedThrottle -= hoverPercentage;

            VesselController.setThrottle(clamp(pidCalculatedThrottle, -1.0, 1.0) * 0.5 + 0.5);

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
            brakingLandingPrediction = Forecast.predictLandPositionWithBraking();
            brakeAltitudePrediction = Forecast.predictImmediateRetrogradeBurnStopAltitude();

            var altitude = VesselController.getAltitude();
            var velocity = VesselController.getVelocity().Length();
            var targetRelative = Target - landingPrediction.Position;
            double missDistance = targetRelative.Length();

            if (currentStage == Stage.HighAltitudeCorrections && missDistance < 500)
            {
                currentStage = Stage.BallisticDescend;
            }
            else if (currentStage == Stage.BallisticDescend && (altitude < 25000 || velocity > 1000))
            {
                VesselController.setThrustPercentage(1.0f);
                currentStage = Stage.ReentryBurn;
            }
            else if (currentStage == Stage.ReentryBurn && velocity < 550)
            {
                VesselController.setThrustPercentage(0.5f);
                currentStage = Stage.BallisticDescend2;
            }
            else if (currentStage == Stage.BallisticDescend2 && brakeAltitudePrediction <= 500.0)
            {
                currentStage = Stage.LandingBurn;
            }
            else if (currentStage == Stage.LandingBurn && altitude <= 20.0 && velocity < 2.0f)
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
