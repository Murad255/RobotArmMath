using MathWorks.MATLAB.NET.Arrays;
using MathWorks.MATLAB.NET.Utility;
using System;
using System.IO;
using System.Reflection;

#if SHARED
[assembly: System.Reflection.AssemblyKeyFile(@"")]
#endif

namespace MatLibCore
{
    public class MatLibTest
    {
        public static void test()
        {
            baseA.Class1 Ktest;
            using (Ktest = new baseA.Class1())
            {
                Ktest.begin();
                double p = Math.PI / 180.0;
                int showFlag = 1;
                MWArray Kres;
                using (Kres = Ktest.robot_arm_I(90 * p, -90 * p, 90 * p, 0, 0, 0, showFlag))
                {
                    for (int i = 0; i < 360; i++)
                    {
                        Ktest.robot_arm_I(i * p, -90 * p, 90 * p, 0, 0, 0, showFlag);
                    }
                }
            }
        }
        public static void test1()
        {
            baseA.Class1 Ktest;
            using (Ktest = new baseA.Class1())
            {
                // Ktest.begin();
                double p = Math.PI / 180.0;
                MWArray Kres;
                //using (Kres = Ktest.forwardKinematics((MWArray)(90 * p), (MWArray)(-90 * p), (MWArray)(90 * p), (MWArray)0, (MWArray)0, (MWArray)0).)
                //{
                //    Console.WriteLine($"Kres = {Kres.ToString()}");
                //    Console.WriteLine($"KresLen = {Kres.ToArray().Length.ToString()}");
                //    Array ar = Kres.ToArray();
                //    //for (int i = 0; i < 360; i++)
                //    //{
                //    //    Kres=Ktest.forwardKinematics((MWArray)(i * p), (MWArray)(-90 * p), (MWArray)(90 * p), (MWArray)0, (MWArray)0, (MWArray)0);
                //    //    Console.WriteLine($"Kres = {Kres.ToString()}");

                //    //}
                //}
                int numArgsOut = 4;
                 MWArray[] Mres = Ktest.forwardKinematics( 4, (MWArray)(0 * p), (MWArray)(-90 * p), (MWArray)(90 * p), (MWArray)0, (MWArray)0, (MWArray)0);
                MWArray[] Mflags = Ktest.CalculateI(3, (MWArray)(0 * p), (MWArray)(-90 * p), (MWArray)(90 * p), (MWArray)0, (MWArray)0, (MWArray)0);

                double r1 = ((MWNumericArray)Mres[0]).ToScalarDouble();
                double r2 = ((MWNumericArray)Mres[1]).ToScalarDouble();
                double r3 = ((MWNumericArray)Mres[2]).ToScalarDouble();
                int f1=  ((MWNumericArray)Mflags[0]).ToScalarInteger();
                int f2 = ((MWNumericArray)Mflags[1]).ToScalarInteger();
                int f3 = ((MWNumericArray)Mflags[2]).ToScalarInteger();


                Console.WriteLine($"x = {r1.ToString()}");
                Console.WriteLine($"y = {r2.ToString()}");
                Console.WriteLine($"z = {r3.ToString()}");
                Console.WriteLine($"A = {Mres[3].ToString()}");
                Console.WriteLine($"f1 = {f1}\tf1 = {f2}\tf1 = {f3}\t");
    

                double[,] resAr = MWtoArray(Mres[3]);
                MWNumericArray MWNresAr = resAr;
                Console.WriteLine($"res = {resAr}");



                MWArray[] Mres2 = Ktest.robot_arm_lin(7, r1, r2, r3, MWNresAr, f1, f2, f3, 0);

                MWNumericArray A1 = (MWNumericArray)Mres2[0];
                MWNumericArray A2 = (MWNumericArray)Mres2[1];
                MWNumericArray A3 = (MWNumericArray)Mres2[2];
                MWNumericArray A4 = (MWNumericArray)Mres2[3];
                MWNumericArray A5 = (MWNumericArray)Mres2[4];
                MWNumericArray A6 = (MWNumericArray)Mres2[5];

                Console.WriteLine($"A1 = {A1}\tA2 = {A2}\tA3 = {A3}\tA4 = {A4}\tA5 = {A5}\tA6 = {A6}");

            }
        }
        public static void test2()
        {
            testEx.TestMatClass Mtest;
            MWArray res;
            using (Mtest = new testEx.TestMatClass())
            {
                using (res = Mtest.plane((MWArray)5, 8))
                {
                    Console.WriteLine($"result = {res.ToString()}");
                    res = Mtest.plane((MWArray)4, 15);
                    Console.WriteLine($"result = {res.ToString()}");
                    res = Mtest.plane((MWArray)1, 5);
                    Console.WriteLine($"result = {res.ToString()}");
                    res = Mtest.plane((MWArray)8, 8);
                    Console.WriteLine($"result = {res.ToString()}");
                    res = Mtest.plane(0.5, 1);
                    Console.WriteLine($"result = {res.ToString()}");
                }
            }
        }

        public static double[,] MWtoArray(MWArray array)
        {
            MWNumericArray Nres = (MWNumericArray)array;
            double[,] res= new double[Nres.Dimensions[0], Nres.Dimensions[1]];

            for (int i= 0; i < Nres.Dimensions[0];i++)
            {
                for (int l = 0; l < Nres.Dimensions[1] ; l++)
                {
                    res[i, l] = Nres[Nres.Dimensions[1] * l + i+1].ToScalarDouble();
                }
            }

            return res;
        }

    }
}
