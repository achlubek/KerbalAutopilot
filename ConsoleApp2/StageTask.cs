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
    class StageTask : IFlightTask
    {
        public StageTask(VesselController vesselController)
        {
            VesselController = vesselController;
        }

        VesselController VesselController;
        public bool update()
        {
            VesselController.activateNextStage();
            return true;
        }
    }
}
