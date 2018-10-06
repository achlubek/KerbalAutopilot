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
    class Forecast
    {

        public class LandingPrediction
        {
            public Vector3 Position;
            public Vector3 Velocity;
            public double TimeLeft;
        }

        public class ApoapsisPeriapsisPrediction
        {
            public double Apoapsis;
            public double Periapsis;
            public double TimeToApoapsis;
            public double TimeToPeriapsis;
        }

        public Forecast(VesselController vesselController)
        {
            VesselController = vesselController;
        }

        VesselController VesselController;

        public LandingPrediction predictLandPosition()
        {
            VesselController.updateBodyPosition();
            var currentPosition = VesselController.getPosition();
            var currentVelocity = VesselController.getVelocity();
            float stepsize = 0.01f;
            float time = 0.0f;
            while (VesselController.getAltitudeAtPoint(currentPosition) > 0)
            {
                currentPosition += currentVelocity * stepsize;
                currentVelocity += VesselController.getGravityAtPoint(currentPosition) * stepsize;
                time += stepsize;
            }
            return new LandingPrediction()
            {
                Position = currentPosition,
                Velocity = currentVelocity,
                TimeLeft = time
            };
        }

        public ApoapsisPeriapsisPrediction predictOrbit()
        {
            VesselController.setReferenceFrameType(VesselController.ReferenceFrameType.ClosestBodyCenter);
            VesselController.updateBodyPosition();
            var currentPosition = VesselController.getPosition();
            var currentVelocity = VesselController.getVelocity();
            float stepsize = 1.0f;

            float time = 0.0f;
            double min = 99999999990.0;
            double max = 0.0;
            double last = VesselController.getAltitudeAtPoint(currentPosition);
            int direction = 0;
            int changes = 0;
            double timeToApo = 0.0;
            double timeToPeri = 0.0;
            while (true)
            {
                var gravity = VesselController.getGravityAtPoint(currentPosition);
                var gravityLength = gravity.Length();
                //Console.WriteLine(gravity.Length());
                currentVelocity += gravity * stepsize;
                currentPosition += currentVelocity * stepsize;
                double newAltitude = VesselController.getAltitudeAtPoint(currentPosition);
                time += stepsize;
                if (newAltitude < min)
                {
                    min = newAltitude;
                    timeToPeri = time;
                }
                if (newAltitude > max)
                {
                    max = newAltitude;
                    timeToApo = time;
                }
                if(direction == 0)
                {
                    direction = newAltitude < last ? -1 : 1;
                    changes++;
                }
                else if(direction == 1 && gravityLength < last)
                {
                    changes++;
                    direction = -1;
                }
                else if(direction == -1 && gravityLength >= last)
                {
                    changes++;
                    direction = 1;
                }
                last = last * 0.999 + 0.001 * gravityLength;
                if (changes > 3) break;
                if (newAltitude < 0) break;
            }
            VesselController.setReferenceFrameType(VesselController.ReferenceFrameType.ClosestBodySurface);
            VesselController.updateBodyPosition();
            return new ApoapsisPeriapsisPrediction()
            {
                Apoapsis = max,
                Periapsis = min,
                TimeToApoapsis = timeToApo,
                TimeToPeriapsis = timeToPeri
            };
        }

        public double predictImmediateRetrogradeBurnStopAltitude()
        {
            VesselController.updateBodyPosition();
            var currentPosition = VesselController.getPosition();
            var currentVelocity = VesselController.getVelocity();
            float stepsize = 0.01f;
            float time = 0.0f;
            bool isVelocityDecreasing = true;
            double lastVelocityMagnitude = currentVelocity.Length();
            while (isVelocityDecreasing)
            {
                var vesselThrust = VesselController.getEnginesAcceleration();
                var normalizedVelocity = currentVelocity;
                normalizedVelocity.Normalize();
                currentPosition += currentVelocity * stepsize;
                currentVelocity += VesselController.getGravityAtPoint(currentPosition) * stepsize;
                currentVelocity += (float)vesselThrust * -normalizedVelocity * stepsize;
                time += stepsize;
                isVelocityDecreasing = currentVelocity.Length() < lastVelocityMagnitude;
                lastVelocityMagnitude = currentVelocity.Length();
            }
            return VesselController.getAltitudeAtPoint(currentPosition);
        }
    }
}
