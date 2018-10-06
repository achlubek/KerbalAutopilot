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
    class WaitForAltitudeTask : IFlightTask
    {
        public WaitForAltitudeTask(VesselController vesselController, VesselDirectionController vesselDirectionController, Forecast forecast, double altitude)
        {
            VesselController = vesselController;
            VesselDirectionController = vesselDirectionController;
            Forecast = forecast;
            Altitude = altitude;
        }

        VesselController VesselController;
        VesselDirectionController VesselDirectionController;
        Forecast Forecast;
        double Altitude;
        
        public bool update()
        {
            VesselDirectionController.update();
            return VesselController.getAltitude() >= Altitude;
        }
    }
}
