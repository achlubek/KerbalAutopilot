﻿using System;
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
    class VesselDirectionController
    {
        public VesselDirectionController(VesselController vesselController)
        {
            VesselController = vesselController;
            targetDirection = new Vector3(0.0f, 1.0f, 0.0f);
            orientationPitchContoller = new PercentageDerivativeController(3.0, 14.0, 0.0);
            orientationYawContoller = new PercentageDerivativeController(3.0, 14.0, 0.0);
            orientationRollContoller = new PercentageDerivativeController(0.4, 4.0, 0.0);
        }

        VesselController VesselController;
        Vector3 targetDirection;
        PercentageDerivativeController orientationPitchContoller;
        PercentageDerivativeController orientationYawContoller;
        PercentageDerivativeController orientationRollContoller;

        public void setTargetDirection(Vector3 dir)
        {
            targetDirection = dir;
        }
        
        static double clamp(double value, double min, double max)
        {
            if (value > max) return max;
            if (value < min) return min;
            return value;
        }

        public double getOnTargetPercentage()
        {
            return Math.Max(0.0, Vector3.Dot(VesselController.getDirection(), targetDirection));
        }

        public void update()
        {
            VesselController.updateBodyPosition();
            var quat = VesselController.getOrientation();

            var shipup = Vector3.Transform(new Vector3(1.0f, 0.0f, 0.0f), quat);
            var shipleft = Vector3.Transform(new Vector3(0.0f, 0.0f, 1.0f), quat);
            var shipforward = Vector3.Transform(new Vector3(0.0f, 1.0f, 0.0f), quat);

            var fdir = shipforward + targetDirection * 1.1f;
            fdir.Normalize();

            float xt = -Vector3.Dot(shipup, fdir);
            float yt = -Vector3.Dot(shipleft, fdir);
            float zt = -Vector3.Dot(shipup, Vector3.UnitX);

            //VesselController.setYaw((float)clamp(xt * 0.1, -0.1, 0.1));
            //VesselController.setPitch((float)clamp(-yt * 0.1, -0.1, 0.1));
            VesselController.setYaw((float)clamp(orientationYawContoller.Calculate(0.0, xt), -1.0, 1.0));
            VesselController.setPitch((float)clamp(-orientationPitchContoller.Calculate(0.0, yt), -1.0, 1.0));
            VesselController.setRoll((float)clamp(orientationRollContoller.Calculate(0.0, zt), -1.0, 1.0));
            //  Console.WriteLine(VesselController.getAngularVelocity().X);
            //  Console.WriteLine(VesselController.getAngularVelocity().Y);
            //  Console.WriteLine(VesselController.getAngularVelocity().Z);
            //  Console.WriteLine(" ");
        }
    }
}
