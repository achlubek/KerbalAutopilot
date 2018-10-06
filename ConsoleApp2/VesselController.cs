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
    class VesselController
    {
        public VesselController(Flight flight, Vessel vessel)
        {
            Flight = flight;
            Vessel = vessel;
            Orbit = Vessel.Orbit;
            refFrame = Orbit.Body.ReferenceFrame;
            nonRotatingRefFrame = Orbit.Body.NonRotatingReferenceFrame;
            Body = Orbit.Body;
            BodyPosition = tupleToVec3(Body.Position(refFrame));
            thrustLimit = 1.0f;
        }

        Flight Flight;
        Vessel Vessel;
        Orbit Orbit;
        ReferenceFrame refFrame;
        ReferenceFrame nonRotatingRefFrame;
        CelestialBody Body;
        Vector3 BodyPosition;
        double bodyRadius;
        float surfaceGravity;
        float vesselMass;
        float vesselAvailableThrust;
        float thrustLimit;
        ReferenceFrameType referenceFrameType = ReferenceFrameType.ClosestBodySurface;

        public enum ReferenceFrameType
        {
            ClosestBodySurface,
            ClosestBodyCenter
        }

        public void setReferenceFrameType(ReferenceFrameType type)
        {
            referenceFrameType = type;
        }

        public void updateBodyPosition()
        {
            BodyPosition = tupleToVec3(Body.Position(refFrame));
            bodyRadius = getDistanceFromBodyCenter() - Flight.SurfaceAltitude;
            surfaceGravity = Body.SurfaceGravity;
            vesselMass = Vessel.Mass;
            vesselAvailableThrust = Vessel.AvailableThrust;
            refFrame = referenceFrameType == ReferenceFrameType.ClosestBodySurface ? Orbit.Body.ReferenceFrame : Orbit.Body.NonRotatingReferenceFrame;
            nonRotatingRefFrame = Orbit.Body.NonRotatingReferenceFrame;
        }

        public void setThrustPercentage(double value)
        {
            Vessel.Parts.Engines.ToList().ForEach((a) => a.ThrustLimit = (float)value);
            thrustLimit = (float)value;
        }

        public float getThrustPercentage()
        {
            return thrustLimit;
        }

        public void setThrottle(double value)
        {
            Vessel.Control.Throttle = (float)value;
        }

        public double getThrottle()
        {
            return (float)Vessel.Control.Throttle;
        }

        public void setPitch(double value)
        {
            Vessel.Control.Pitch = (float)value;
        }

        public void setYaw(double value)
        {
            Vessel.Control.Yaw = (float)value;
        }

        public void setRoll(double value)
        {
            Vessel.Control.Roll = (float)value;
        }

        public Vector3 getAngularVelocity()
        {
            return tupleToVec3(Vessel.AngularVelocity(refFrame));
        }

        static Vector3 tupleToVec3(Tuple<double, double, double> val)
        {
            return new Vector3((float)val.Item1, (float)val.Item2, (float)val.Item3);
        }

        static Quaternion tupleToQuat(Tuple<double, double, double, double> val)
        {
            return new Quaternion((float)val.Item1, (float)val.Item2, (float)val.Item3, (float)val.Item4);
        }

        public Vector3 getDirection()
        {
            return tupleToVec3(Vessel.Direction(refFrame));
        }

        public void setLandingGearState(bool extended)
        {
            if (!Vessel.Control.Gear && extended) Vessel.Control.Gear = true;
            if (Vessel.Control.Gear && !extended) Vessel.Control.Gear = false;
        }

        public Quaternion getOrientation()
        {
            return tupleToQuat(Vessel.Rotation(refFrame));
        }

        public Vector3 getPosition()
        {
            return tupleToVec3(Vessel.Position(refFrame));
        }

        public Vector3 getVelocity()
        {
            return tupleToVec3(Vessel.Velocity(refFrame));
        }

        public Vector3 getOrbitalVelocity()
        {
            return tupleToVec3(Vessel.Velocity(nonRotatingRefFrame));
        }

        public Vector3 getGravity()
        {
            var gravityDirection = BodyPosition - getPosition();
            gravityDirection.Normalize();
            return surfaceGravity * gravityDirection;
        }

        public double calculateGravityStrengthAtAltitude(double altitude)
        {
            //x = (surface * equatorial ^ 2) / distance ^ 2
            double radiusmm = bodyRadius * 0.01;
            double distance = radiusmm + altitude * 0.01;
            return (surfaceGravity * (radiusmm * radiusmm)) / (distance * distance);
        }

        public double calculateGravityStrength()
        {
            return calculateGravityStrengthAtAltitude(getAltitude());
        }

        public Vector3 getGravityAtPoint(Vector3 point)
        {
            var gravityDirection = BodyPosition - point;
            gravityDirection.Normalize();
            return (float)calculateGravityStrengthAtAltitude((BodyPosition - point).Length() - bodyRadius) * gravityDirection;
        }

        public double calculateOrbitalVelocityAtAltitude(double altitude)
        {
            return Math.Sqrt(calculateGravityStrengthAtAltitude(altitude) * (altitude + bodyRadius));
        }

        public double getAltitude()
        {
            var box = Vessel.BoundingBox(refFrame);
            double x = Math.Abs(box.Item1.Item1 - box.Item2.Item1);
            double y = Math.Abs(box.Item1.Item2 - box.Item2.Item2);
            double z = Math.Abs(box.Item1.Item3 - box.Item2.Item3);
            double maxbound = Math.Max(Math.Max(x, y), z) * 0.5;
            return Flight.SurfaceAltitude - maxbound;
        }

        public double getDistanceFromBodyCenter()
        {
            return (BodyPosition - getPosition()).Length();
        }

        public double getAltitudeAtPoint(Vector3 point)
        {
            return (BodyPosition - point).Length() - bodyRadius;
        }

        public double getEnginesAcceleration()
        {
            double gravity = surfaceGravity;
            return ((vesselAvailableThrust * thrustLimit) / (vesselMass * gravity)) * gravity;
        }

        public double getEnginesAccelerationPrediction(double percentOfPower)
        {
            double gravity = surfaceGravity;
            return ((vesselAvailableThrust * percentOfPower) / (vesselMass * gravity)) * gravity;
        }

        public void activateNextStage()
        {
            Vessel.Control.ActivateNextStage();
        }
    }
}
