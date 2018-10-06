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
        }

        VesselController VesselController;
        VesselDirectionController VesselDirectionController;
        Forecast Forecast;

        public bool update()
        {
            var brakeAltitudePrediction = Forecast.predictImmediateRetrogradeBurnStopAltitude();

            if (brakeAltitudePrediction <= 0.0)
            {
                VesselController.setThrottle(1.0f);
            }
            else
            {
                VesselController.setThrottle(VesselController.getThrottle() * 0.8f);
            }

            var vesselVelocity = VesselController.getVelocity();
            var vesselVelocityNormalized = vesselVelocity;

            VesselDirectionController.setTargetDirection(-vesselVelocityNormalized);
            VesselDirectionController.update();

            if (VesselController.getAltitude() < 500)
            {
                VesselController.setLandingGearState(true);
            }
            
            Console.WriteLine("Brake prediction {0}", brakeAltitudePrediction);

            return vesselVelocity.Length() < 2.0;
        }
    }
}
