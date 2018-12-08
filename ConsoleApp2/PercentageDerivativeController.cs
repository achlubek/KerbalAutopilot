using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using System.Diagnostics;
using System.Threading;
using SharpDX;
using SharpDX.Mathematics;

namespace ConsoleApp2
{
    class PercentageDerivativeController
    {
        public PercentageDerivativeController(double kp, double kd, double ki)
        {
            KD = kd;
            KP = kp;
            KI = ki;
        }

        double error_prior = 0.0;
        double integral = 0.0;
        double KP = 0.5;
        double KD = 0.5;
        double KI = 0.5;
        Stopwatch stopWatch = new Stopwatch();

        public double Calculate(double target, double actual)
        {
            double elapsed = (double)Math.Max(1, stopWatch.ElapsedMilliseconds) / 1000.0;
            stopWatch.Reset();
            stopWatch.Start();
            double error = target - actual;
            double derivative = (error - error_prior) / elapsed;
            integral += error * elapsed;
            double output = KP * error + KD * derivative + KI * integral;
            error_prior = error;
            return output;
        }
    }

    class PercentageDerivativeControllerVec3
    {
        public PercentageDerivativeControllerVec3(float kp, float kd, float ki)
        {
            KD = kd;
            KP = kp;
            KI = ki;
        }

        Vector3 error_prior = Vector3.Zero;
        Vector3 integral = Vector3.Zero;
        float KP = 0.5f;
        float KD = 0.5f;
        float KI = 0.5f;
        Stopwatch stopWatch = new Stopwatch();

        public Vector3 Calculate(Vector3 target, Vector3 actual)
        {
            float elapsed = (float)Math.Max(1, stopWatch.ElapsedMilliseconds) / 1000.0f;
            stopWatch.Reset();
            stopWatch.Start();
            Vector3 error = target - actual;
            Vector3 derivative = (error - error_prior) / elapsed;
            integral += error * elapsed;
            Vector3 output = KP * error + KD * derivative + KI * integral;
            error_prior = error;
            return output;
        }
    }
}
