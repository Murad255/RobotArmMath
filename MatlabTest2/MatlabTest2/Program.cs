using MathWorks.MATLAB.NET.Arrays;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MathWorks.MATLAB.NET.Utility;

namespace MatlabTest2
{
    class Program
    {
        static void Main(string[] args)
        {
            //TestMatClass matClass = new TestMatClass();

            //MWArray res2 = matClass.plane((MWArray)"5", (MWArray)"6");
            baseA.Class1 Ktest = new baseA.Class1();
            testEx.TestMatClass Mtest = new testEx.TestMatClass();
            MWArray res;
            using (Ktest = new baseA.Class1()) using (Mtest = new testEx.TestMatClass())
            {
                res = Mtest.plane((MWArray)5, (MWArray)8);
                Console.WriteLine($"result = {res.ToString()}");
                res = Mtest.plane((MWArray)4, (MWArray)15);
                Console.WriteLine($"result = {res.ToString()}");
                res = Mtest.plane((MWArray)1, (MWArray)5);
                Console.WriteLine($"result = {res.ToString()}");
                res = Mtest.plane((MWArray)8, (MWArray)8);
                Console.WriteLine($"result = {res.ToString()}");
                res = Mtest.plane((MWArray)0.5, (MWArray)1);
                Console.WriteLine($"result = {res.ToString()}");

                
                double p = Math.PI / 180.0;
                int showFlag = 1;
                MWArray Kres = Ktest.forwardKinematics((MWArray)(90 * p), (MWArray)(-90 * p), (MWArray)(90 * p), (MWArray)0, (MWArray)0, (MWArray)0);
      
                Console.WriteLine($"Kres1 = {Kres.ToString()}");
                Console.WriteLine($"Kres2 = {Kres.ToString()}");



                Ktest.begin();

                //for (int i = 0; i < 360; i++)
                //{
                //    Ktest.robot_arm_I((MWArray)(i * p), (MWArray)(-90 * p), (MWArray)(90 * p), (MWArray)0, (MWArray)0, (MWArray)0, showFlag);
                //}
                Console.Read();

                Kres.Dispose();

            } 
        }
    }
}
