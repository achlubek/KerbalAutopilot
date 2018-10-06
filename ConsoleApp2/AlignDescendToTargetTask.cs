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
    class AlignDescendToTargetTask : IFlightTask
    {
        public AlignDescendToTargetTask(VesselController vesselController, VesselDirectionController vesselDirectionController, Forecast forecast, Vector3 target)
        {
            VesselController = vesselController;
            VesselDirectionController = vesselDirectionController;
            Forecast = forecast;
            Target = target;
        }

        VesselController VesselController;
        VesselDirectionController VesselDirectionController;
        Forecast Forecast;
        Vector3 Target;

        double lastMissDistance = 9999999999.0;

        public bool update()
        {
            var landingPrediction = Forecast.predictLandPosition();
            var brakeAltitudePrediction = Forecast.predictImmediateRetrogradeBurnStopAltitude();
            var targetRelative = Target - landingPrediction.Position;
            var targetRelativeNormalized = targetRelative;
            targetRelativeNormalized.Normalize();
            
            float minimalCorrectiveThrottle = 0.0f;
            
            var desiredDirection = targetRelativeNormalized;
            desiredDirection.Normalize();
            VesselDirectionController.setTargetDirection(desiredDirection);
            minimalCorrectiveThrottle = (float)VesselDirectionController.getOnTargetPercentage() *
                Math.Min(1.0f, (float)Math.Pow(targetRelative.Length() * 0.0008f, 2.0f)) * 0.1f;
            
            VesselDirectionController.update();

            Console.WriteLine("Prediction mismatch {0}", targetRelative.Length());

            if ((targetRelative.Length() < 1000.0 && lastMissDistance < targetRelative.Length()) || targetRelative.Length() < 10.0)
            {
                VesselController.setThrottle(0.0);
                return true;
            }
            
            lastMissDistance = targetRelative.Length();

            VesselController.setThrottle(minimalCorrectiveThrottle);

            return false;
        }
    }
}
