/*
* MATLAB Compiler: 7.0 (R2018b)
* Date: Fri Nov 20 13:23:34 2020
* Arguments:
* "-B""macro_default""-W""dotnet:simulation,Class1,4.0,private""-T""link:lib""-d""G:\Googl
* eDisck\ãçñ¡ _online\SIENCE\Simulation_matlab\robot_arm\for_testing""-v""class{Class1:G:\
* GoogleDisck\ãçñ¡ _online\SIENCE\Simulation_matlab\robot_arm.m}"
*/
using System;
using System.Reflection;
using System.IO;
using MathWorks.MATLAB.NET.Arrays;
using MathWorks.MATLAB.NET.Utility;

#if SHARED
[assembly: System.Reflection.AssemblyKeyFile(@"")]
#endif

namespace simulation
{

  /// <summary>
  /// The Class1 class provides a CLS compliant, MWArray interface to the MATLAB
  /// functions contained in the files:
  /// <newpara></newpara>
  /// G:\GoogleDisck\ó÷¸áà_online\SIENCE\Simulation_matlab\robot_arm.m
  /// </summary>
  /// <remarks>
  /// @Version 4.0
  /// </remarks>
  public class Class1 : IDisposable
  {
    #region Constructors

    /// <summary internal= "true">
    /// The static constructor instantiates and initializes the MATLAB Runtime instance.
    /// </summary>
    static Class1()
    {
      if (MWMCR.MCRAppInitialized)
      {
        try
        {
          Assembly assembly= Assembly.GetExecutingAssembly();

          string ctfFilePath= assembly.Location;

          int lastDelimiter= ctfFilePath.LastIndexOf(@"\");

          ctfFilePath= ctfFilePath.Remove(lastDelimiter, (ctfFilePath.Length - lastDelimiter));

          string ctfFileName = "simulation.ctf";

          Stream embeddedCtfStream = null;

          String[] resourceStrings = assembly.GetManifestResourceNames();

          foreach (String name in resourceStrings)
          {
            if (name.Contains(ctfFileName))
            {
              embeddedCtfStream = assembly.GetManifestResourceStream(name);
              break;
            }
          }
          mcr= new MWMCR("",
                         ctfFilePath, embeddedCtfStream, true);
        }
        catch(Exception ex)
        {
          ex_ = new Exception("MWArray assembly failed to be initialized", ex);
        }
      }
      else
      {
        ex_ = new ApplicationException("MWArray assembly could not be initialized");
      }
    }


    /// <summary>
    /// Constructs a new instance of the Class1 class.
    /// </summary>
    public Class1()
    {
      if(ex_ != null)
      {
        throw ex_;
      }
    }


    #endregion Constructors

    #region Finalize

    /// <summary internal= "true">
    /// Class destructor called by the CLR garbage collector.
    /// </summary>
    ~Class1()
    {
      Dispose(false);
    }


    /// <summary>
    /// Frees the native resources associated with this object
    /// </summary>
    public void Dispose()
    {
      Dispose(true);

      GC.SuppressFinalize(this);
    }


    /// <summary internal= "true">
    /// Internal dispose function
    /// </summary>
    protected virtual void Dispose(bool disposing)
    {
      if (!disposed)
      {
        disposed= true;

        if (disposing)
        {
          // Free managed resources;
        }

        // Free native resources
      }
    }


    #endregion Finalize

    #region Methods

    /// <summary>
    /// Provides a void output, 0-input MWArrayinterface to the robot_arm MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ROBOT_ARM Summary of this function goes here
    /// Detailed explanation goes here.
    /// </remarks>
    ///
    public void robot_arm()
    {
      mcr.EvaluateFunction(0, "robot_arm", new MWArray[]{});
    }


    /// <summary>
    /// Provides a void output, 1-input MWArrayinterface to the robot_arm MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ROBOT_ARM Summary of this function goes here
    /// Detailed explanation goes here.
    /// </remarks>
    /// <param name="q1">Input argument #1</param>
    ///
    public void robot_arm(MWArray q1)
    {
      mcr.EvaluateFunction(0, "robot_arm", q1);
    }


    /// <summary>
    /// Provides a void output, 2-input MWArrayinterface to the robot_arm MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ROBOT_ARM Summary of this function goes here
    /// Detailed explanation goes here.
    /// </remarks>
    /// <param name="q1">Input argument #1</param>
    /// <param name="q2">Input argument #2</param>
    ///
    public void robot_arm(MWArray q1, MWArray q2)
    {
      mcr.EvaluateFunction(0, "robot_arm", q1, q2);
    }


    /// <summary>
    /// Provides a void output, 3-input MWArrayinterface to the robot_arm MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ROBOT_ARM Summary of this function goes here
    /// Detailed explanation goes here.
    /// </remarks>
    /// <param name="q1">Input argument #1</param>
    /// <param name="q2">Input argument #2</param>
    /// <param name="q3">Input argument #3</param>
    ///
    public void robot_arm(MWArray q1, MWArray q2, MWArray q3)
    {
      mcr.EvaluateFunction(0, "robot_arm", q1, q2, q3);
    }


    /// <summary>
    /// Provides a void output, 4-input MWArrayinterface to the robot_arm MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ROBOT_ARM Summary of this function goes here
    /// Detailed explanation goes here.
    /// </remarks>
    /// <param name="q1">Input argument #1</param>
    /// <param name="q2">Input argument #2</param>
    /// <param name="q3">Input argument #3</param>
    /// <param name="q4">Input argument #4</param>
    ///
    public void robot_arm(MWArray q1, MWArray q2, MWArray q3, MWArray q4)
    {
      mcr.EvaluateFunction(0, "robot_arm", q1, q2, q3, q4);
    }


    /// <summary>
    /// Provides a void output, 5-input MWArrayinterface to the robot_arm MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ROBOT_ARM Summary of this function goes here
    /// Detailed explanation goes here.
    /// </remarks>
    /// <param name="q1">Input argument #1</param>
    /// <param name="q2">Input argument #2</param>
    /// <param name="q3">Input argument #3</param>
    /// <param name="q4">Input argument #4</param>
    /// <param name="q5">Input argument #5</param>
    ///
    public void robot_arm(MWArray q1, MWArray q2, MWArray q3, MWArray q4, MWArray q5)
    {
      mcr.EvaluateFunction(0, "robot_arm", q1, q2, q3, q4, q5);
    }


    /// <summary>
    /// Provides a void output, 6-input MWArrayinterface to the robot_arm MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ROBOT_ARM Summary of this function goes here
    /// Detailed explanation goes here.
    /// </remarks>
    /// <param name="q1">Input argument #1</param>
    /// <param name="q2">Input argument #2</param>
    /// <param name="q3">Input argument #3</param>
    /// <param name="q4">Input argument #4</param>
    /// <param name="q5">Input argument #5</param>
    /// <param name="q6">Input argument #6</param>
    ///
    public void robot_arm(MWArray q1, MWArray q2, MWArray q3, MWArray q4, MWArray q5, 
                    MWArray q6)
    {
      mcr.EvaluateFunction(0, "robot_arm", q1, q2, q3, q4, q5, q6);
    }


    /// <summary>
    /// Provides the standard 0-input MWArray interface to the robot_arm MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ROBOT_ARM Summary of this function goes here
    /// Detailed explanation goes here.
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] robot_arm(int numArgsOut)
    {
      return mcr.EvaluateFunction(numArgsOut, "robot_arm", new MWArray[]{});
    }


    /// <summary>
    /// Provides the standard 1-input MWArray interface to the robot_arm MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ROBOT_ARM Summary of this function goes here
    /// Detailed explanation goes here.
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="q1">Input argument #1</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] robot_arm(int numArgsOut, MWArray q1)
    {
      return mcr.EvaluateFunction(numArgsOut, "robot_arm", q1);
    }


    /// <summary>
    /// Provides the standard 2-input MWArray interface to the robot_arm MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ROBOT_ARM Summary of this function goes here
    /// Detailed explanation goes here.
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="q1">Input argument #1</param>
    /// <param name="q2">Input argument #2</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] robot_arm(int numArgsOut, MWArray q1, MWArray q2)
    {
      return mcr.EvaluateFunction(numArgsOut, "robot_arm", q1, q2);
    }


    /// <summary>
    /// Provides the standard 3-input MWArray interface to the robot_arm MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ROBOT_ARM Summary of this function goes here
    /// Detailed explanation goes here.
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="q1">Input argument #1</param>
    /// <param name="q2">Input argument #2</param>
    /// <param name="q3">Input argument #3</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] robot_arm(int numArgsOut, MWArray q1, MWArray q2, MWArray q3)
    {
      return mcr.EvaluateFunction(numArgsOut, "robot_arm", q1, q2, q3);
    }


    /// <summary>
    /// Provides the standard 4-input MWArray interface to the robot_arm MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ROBOT_ARM Summary of this function goes here
    /// Detailed explanation goes here.
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="q1">Input argument #1</param>
    /// <param name="q2">Input argument #2</param>
    /// <param name="q3">Input argument #3</param>
    /// <param name="q4">Input argument #4</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] robot_arm(int numArgsOut, MWArray q1, MWArray q2, MWArray q3, 
                         MWArray q4)
    {
      return mcr.EvaluateFunction(numArgsOut, "robot_arm", q1, q2, q3, q4);
    }


    /// <summary>
    /// Provides the standard 5-input MWArray interface to the robot_arm MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ROBOT_ARM Summary of this function goes here
    /// Detailed explanation goes here.
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="q1">Input argument #1</param>
    /// <param name="q2">Input argument #2</param>
    /// <param name="q3">Input argument #3</param>
    /// <param name="q4">Input argument #4</param>
    /// <param name="q5">Input argument #5</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] robot_arm(int numArgsOut, MWArray q1, MWArray q2, MWArray q3, 
                         MWArray q4, MWArray q5)
    {
      return mcr.EvaluateFunction(numArgsOut, "robot_arm", q1, q2, q3, q4, q5);
    }


    /// <summary>
    /// Provides the standard 6-input MWArray interface to the robot_arm MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ROBOT_ARM Summary of this function goes here
    /// Detailed explanation goes here.
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="q1">Input argument #1</param>
    /// <param name="q2">Input argument #2</param>
    /// <param name="q3">Input argument #3</param>
    /// <param name="q4">Input argument #4</param>
    /// <param name="q5">Input argument #5</param>
    /// <param name="q6">Input argument #6</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] robot_arm(int numArgsOut, MWArray q1, MWArray q2, MWArray q3, 
                         MWArray q4, MWArray q5, MWArray q6)
    {
      return mcr.EvaluateFunction(numArgsOut, "robot_arm", q1, q2, q3, q4, q5, q6);
    }



    /// <summary>
    /// This method will cause a MATLAB figure window to behave as a modal dialog box.
    /// The method will not return until all the figure windows associated with this
    /// component have been closed.
    /// </summary>
    /// <remarks>
    /// An application should only call this method when required to keep the
    /// MATLAB figure window from disappearing.  Other techniques, such as calling
    /// Console.ReadLine() from the application should be considered where
    /// possible.</remarks>
    ///
    public void WaitForFiguresToDie()
    {
      mcr.WaitForFiguresToDie();
    }



    #endregion Methods

    #region Class Members

    private static MWMCR mcr= null;

    private static Exception ex_= null;

    private bool disposed= false;

    #endregion Class Members
  }
}
