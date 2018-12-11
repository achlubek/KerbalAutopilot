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
    class MaintainVerticalVelocityTask : IFlightTask
    {
        public MaintainVerticalVelocityTask(VesselController vesselController, VesselDirectionController vesselDirectionController, float targetVelocity)
        {
            VesselController = vesselController;
            VesselDirectionController = vesselDirectionController;
            LandingSpeedPID = new PercentageDerivativeController(1.0, 0.0, 0.0);
            TargetVelocity = targetVelocity;
        }

        VesselController VesselController;
        VesselDirectionController VesselDirectionController;
        PercentageDerivativeController LandingSpeedPID;
        double TargetVelocity;

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

        public bool update()
        {
            var vesselVelocity = VesselController.getVelocity();
            
            var downDirection = VesselController.getGravity();
            downDirection.Normalize();

            var velocity = Vector3.Dot(vesselVelocity, -downDirection);

            VesselDirectionController.setTargetDirection(-downDirection);
            VesselDirectionController.update();

            var pidCalculatedThrottle = LandingSpeedPID.Calculate(TargetVelocity, velocity);
            Console.WriteLine("PID Result : {0}", pidCalculatedThrottle);
            
            double hoverPercentage = predictThrustPercentageForAcceleration(VesselController.getGravity().Length()) * 2.0 - 1.0;

            pidCalculatedThrottle -= hoverPercentage;

            VesselController.setThrottle(clamp(pidCalculatedThrottle, -1.0, 1.0) * 0.5 + 0.5);

            return false;
        }
    }
}
