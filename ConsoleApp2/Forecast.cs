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

        private double calculateDynamicPressure(double altitude, double velocityMagnitude)
        {
            double airDensity = getAirDensityPercentage(altitude);
            if (airDensity < 0.0 || airDensity > 1.0) airDensity = 0.0;
            return 0.5f * airDensity * velocityMagnitude * velocityMagnitude * 0.0001;
        }

        private double getAirDensityPercentage(double altitude)
        {
            double atmosphereHeight = (double)VesselController.getClosestBodyAtmosphereHeight();
            if (atmosphereHeight < 0.0) return 0.0;
            double percentage = altitude / atmosphereHeight;
            return 1.0 - Math.Pow(percentage, 0.333);
        }

        public LandingPrediction predictLandPosition()
        {
            VesselController.updateBodyPosition();
            var currentPosition = VesselController.getPosition();
            var currentVelocity = VesselController.getVelocity();
            float stepsize = 0.01f;
            float time = 0.0f;
            while (VesselController.getAltitudeAtPoint(currentPosition) > 0)
            {
                var normalizedVelocity = currentVelocity;
                normalizedVelocity.Normalize();
                currentPosition += currentVelocity * stepsize;
                currentVelocity += -normalizedVelocity * stepsize * (float)VesselController.getDrag() 
                    * (float)calculateDynamicPressure(VesselController.getAltitudeAtPoint(currentPosition), currentVelocity.Length());
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

        public LandingPrediction predictLandPositionWithBraking()
        {
            VesselController.updateBodyPosition();
            var currentPosition = VesselController.getPosition();
            var currentVelocity = VesselController.getVelocity();
            float stepsize = 0.01f;
            float time = 0.0f;
            bool isVelocityDecreasing = true;
            double lastVelocityMagnitude = currentVelocity.Length();
            while (VesselController.getAltitudeAtPoint(currentPosition) > 0 && isVelocityDecreasing)
            {
                var vesselThrust = VesselController.getEnginesAcceleration();
                var normalizedVelocity = currentVelocity;
                normalizedVelocity.Normalize();
                currentPosition += currentVelocity * stepsize;
                currentVelocity += (float)vesselThrust * -normalizedVelocity * stepsize;
                currentVelocity += -normalizedVelocity * stepsize * (float)VesselController.getDrag()
                    * (float)calculateDynamicPressure(VesselController.getAltitudeAtPoint(currentPosition), currentVelocity.Length());
                currentVelocity += VesselController.getGravityAtPoint(currentPosition) * stepsize;
                time += stepsize;
                isVelocityDecreasing = currentVelocity.Length() < lastVelocityMagnitude;
                lastVelocityMagnitude = currentVelocity.Length();
            }
            while (VesselController.getAltitudeAtPoint(currentPosition) > 0)
            {
                var normalizedVelocity = currentVelocity;
                normalizedVelocity.Normalize();
                currentPosition += currentVelocity * stepsize;
                currentVelocity += -normalizedVelocity * stepsize * (float)VesselController.getDrag()
                    * (float)calculateDynamicPressure(VesselController.getAltitudeAtPoint(currentPosition), currentVelocity.Length());
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
                currentVelocity += (float)vesselThrust * -normalizedVelocity * stepsize;
                currentVelocity += -normalizedVelocity * stepsize * (float)VesselController.getDrag()
                    * (float)calculateDynamicPressure(VesselController.getAltitudeAtPoint(currentPosition), currentVelocity.Length());
                currentVelocity += VesselController.getGravityAtPoint(currentPosition) * stepsize;
                time += stepsize;
                isVelocityDecreasing = currentVelocity.Length() < lastVelocityMagnitude;
                lastVelocityMagnitude = currentVelocity.Length();
            }
            return VesselController.getAltitudeAtPoint(currentPosition);
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
    }
}
