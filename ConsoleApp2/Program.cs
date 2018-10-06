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
    class Program
    {

        static void Main(string[] args)
        {
            using (var connection = new Connection(
                      name: "My Example Program",
                      address: IPAddress.Parse("127.0.0.1"),
                      rpcPort: 50000,
                      streamPort: 50001))
            {
                var krpc = connection.KRPC();
                Console.WriteLine(krpc.GetStatus().Version);


                var spaceCenter = connection.SpaceCenter();
                var vessel = spaceCenter.ActiveVessel;
                var flightInfo = vessel.Flight();


                Console.WriteLine(flightInfo.MeanAltitude);

                vessel.Control.SAS = false;
              //  vessel.Control.SASMode = SASMode.Retrograde;

                var refFrame = vessel.Orbit.Body.ReferenceFrame;

                var vesselPosition = vessel.Position(refFrame);

                double accelerationWithGravity = (vessel.AvailableThrust / (vessel.Mass * vessel.Orbit.Body.SurfaceGravity)) * vessel.Orbit.Body.SurfaceGravity;
                Console.WriteLine("acc " + accelerationWithGravity.ToString());
                //    Console.Read();
                /*
                IFlightTask task = new HoldAltitudeTask(1000.0);

                while (true)
                {
                    double error = task.update(flightInfo, vessel);
                    if (error < 1.0) break;
                }

                task = new HoldVerticalVelocityTask(-25.0);

                while (true)
                {
                    double error = task.update(flightInfo, vessel);
                    ((HoldVerticalVelocityTask)task).targetVelocity = -(flightInfo.SurfaceAltitude * flightInfo.SurfaceAltitude) * 0.0002;
                 //   if (flightInfo.MeanAltitude < 200.0) break;
                }

                task = new HoldVerticalVelocityTask(-2.0);

                while (true)
                {
                    double error = task.update(flightInfo, vessel);
                }
                */
                /*
                List<IFlightTask> tasks = new List<IFlightTask> { new SuicideLandTask() };
                int activeTask = 0;
                while (true)
                {
                    var task = tasks[activeTask];
                    bool completed = task.update(flightInfo, vessel);
                    if (completed) activeTask++;
                    if (activeTask >= tasks.Count) break;
                }
                vessel.Control.Throttle = 0.0f;
                */

                VesselController VC = new VesselController(flightInfo, vessel);
                Forecast F = new Forecast(VC);
                /* 
                 while (true)
                 {
                     var landingPrediction = F.predictLandPosition();
                     var brakeAltitudePrediction = F.predictImmediateRetrogradeBurnStopAltitude();
                     Console.WriteLine("Time to hit {0}, Brake altitude {1}", landingPrediction.TimeLeft, brakeAltitudePrediction);
                     if (brakeAltitudePrediction <= 1.0) vessel.Control.Throttle = 1.0f; else vessel.Control.Throttle = 0.0f;
                   //  System.Threading.Thread.Sleep(30);
                 }*/
                VesselDirectionController VDC = new VesselDirectionController(VC);
                var target = new Vector3(160414.82635621f, -525.786447726701f, -578229.703053695f);
                // var target = new Vector3(159779.889438512f, -1018.08311045618f, -578408.695050623f);
                //var target = new Vector3(161270.085206315f, -397.490824863315f, -577920.690812992f);
                 LandOnTargetTask landTask = new LandOnTargetTask(VC, VDC, F, target);
                /*while (true)
                {
                    landTask.update();
                    System.Threading.Thread.Sleep(30);
                }*/
                /*
                VC.updateBodyPosition();
                while (true)
                {
                    var apoperi = F.predictOrbit();
                    Console.WriteLine("Apoapsis {0} Periapsis {1}", apoperi.Apoapsis, apoperi.Periapsis);
                    System.Threading.Thread.Sleep(30);
                }*/
                AscendToOrbitTask ascend = new AscendToOrbitTask(VC, VDC, F, 75000);
                AscendHalfVectorTask ascendHalf = new AscendHalfVectorTask(VC, VDC, F, 75000);
                AscendStraightUpTask ascendStraight = new AscendStraightUpTask(VC, VDC, F, 75000);
                WaitForAltitudeTask waitTask = new WaitForAltitudeTask(VC, VDC, F, 70000);
                StageTask stageTask = new StageTask(VC);
                /*while (true)
                {
                    ascend.update();
                    System.Threading.Thread.Sleep(30);
                }*/
                //List<IFlightTask> tasks = new List<IFlightTask> { stageTask, ascendStraight, waitTask, stageTask, landTask };
                List<IFlightTask> tasks = new List<IFlightTask> { stageTask, ascendStraight, waitTask, landTask };
                int activeTask = 0;
                while (true)
                {
                    var task = tasks[activeTask];
                    bool completed = task.update();
                    if (completed) activeTask++;
                    if (activeTask >= tasks.Count) break;
                    System.Threading.Thread.Sleep(30);
                }
            }
        }
    }
}
