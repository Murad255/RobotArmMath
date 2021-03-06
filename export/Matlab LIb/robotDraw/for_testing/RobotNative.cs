/*
* MATLAB Compiler: 7.0 (R2018b)
* Date: Wed Nov 18 02:24:04 2020
* Arguments:
* "-B""macro_default""-W""dotnet:robot_arm,Robot,4.0,private""-T""link:lib""-d""G:\GoogleD
* isck\?????_online\SIENCE\Simulation_matlab\robotDraw\for_testing""-v""class{Robot:G:\Goo
* gleDisck\?????_online\SIENCE\Simulation_matlab\robot_arm.m}"
*/
using System;
using System.Reflection;
using System.IO;
using MathWorks.MATLAB.NET.Arrays;
using MathWorks.MATLAB.NET.Utility;

#if SHARED
[assembly: System.Reflection.AssemblyKeyFile(@"")]
#endif

namespace robot_armNative
{

  /// <summary>
  /// The Robot class provides a CLS compliant, Object (native) interface to the MATLAB
  /// functions contained in the files:
  /// <newpara></newpara>
  /// G:\GoogleDisck\?????_online\SIENCE\Simulation_matlab\robot_arm.m
  /// </summary>
  /// <remarks>
  /// @Version 4.0
  /// </remarks>
  public class Robot : IDisposable
  {
    #region Constructors

    /// <summary internal= "true">
    /// The static constructor instantiates and initializes the MATLAB Runtime instance.
    /// </summary>
    static Robot()
    {
      if (MWMCR.MCRAppInitialized)
      {
        try
        {
          Assembly assembly= Assembly.GetExecutingAssembly();

          string ctfFilePath= assembly.Location;

          int lastDelimiter= ctfFilePath.LastIndexOf(@"\");

          ctfFilePath= ctfFilePath.Remove(lastDelimiter, (ctfFilePath.Length - lastDelimiter));

          string ctfFileName = "robot_arm.ctf";

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
    /// Constructs a new instance of the Robot class.
    /// </summary>
    public Robot()
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
    ~Robot()
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
    /// Provides a void output, 0-input Objectinterface to the robot_arm MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ROBOT_ARM Summary of this function goes here
    /// Detailed explanation goes here
    /// </remarks>
    ///
    public void robot_arm()
    {
      mcr.EvaluateFunction(0, "robot_arm", new Object[]{});
    }


    /// <summary>
    /// Provides a void output, 1-input Objectinterface to the robot_arm MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ROBOT_ARM Summary of this function goes here
    /// Detailed explanation goes here
    /// </remarks>
    /// <param name="q1">Input argument #1</param>
    ///
    public void robot_arm(Object q1)
    {
      mcr.EvaluateFunction(0, "robot_arm", q1);
    }


    /// <summary>
    /// Provides a void output, 2-input Objectinterface to the robot_arm MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ROBOT_ARM Summary of this function goes here
    /// Detailed explanation goes here
    /// </remarks>
    /// <param name="q1">Input argument #1</param>
    /// <param name="q2">Input argument #2</param>
    ///
    public void robot_arm(Object q1, Object q2)
    {
      mcr.EvaluateFunction(0, "robot_arm", q1, q2);
    }


    /// <summary>
    /// Provides a void output, 3-input Objectinterface to the robot_arm MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ROBOT_ARM Summary of this function goes here
    /// Detailed explanation goes here
    /// </remarks>
    /// <param name="q1">Input argument #1</param>
    /// <param name="q2">Input argument #2</param>
    /// <param name="q3">Input argument #3</param>
    ///
    public void robot_arm(Object q1, Object q2, Object q3)
    {
      mcr.EvaluateFunction(0, "robot_arm", q1, q2, q3);
    }


    /// <summary>
    /// Provides a void output, 4-input Objectinterface to the robot_arm MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ROBOT_ARM Summary of this function goes here
    /// Detailed explanation goes here
    /// </remarks>
    /// <param name="q1">Input argument #1</param>
    /// <param name="q2">Input argument #2</param>
    /// <param name="q3">Input argument #3</param>
    /// <param name="q4">Input argument #4</param>
    ///
    public void robot_arm(Object q1, Object q2, Object q3, Object q4)
    {
      mcr.EvaluateFunction(0, "robot_arm", q1, q2, q3, q4);
    }


    /// <summary>
    /// Provides a void output, 5-input Objectinterface to the robot_arm MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ROBOT_ARM Summary of this function goes here
    /// Detailed explanation goes here
    /// </remarks>
    /// <param name="q1">Input argument #1</param>
    /// <param name="q2">Input argument #2</param>
    /// <param name="q3">Input argument #3</param>
    /// <param name="q4">Input argument #4</param>
    /// <param name="q5">Input argument #5</param>
    ///
    public void robot_arm(Object q1, Object q2, Object q3, Object q4, Object q5)
    {
      mcr.EvaluateFunction(0, "robot_arm", q1, q2, q3, q4, q5);
    }


    /// <summary>
    /// Provides a void output, 6-input Objectinterface to the robot_arm MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ROBOT_ARM Summary of this function goes here
    /// Detailed explanation goes here
    /// </remarks>
    /// <param name="q1">Input argument #1</param>
    /// <param name="q2">Input argument #2</param>
    /// <param name="q3">Input argument #3</param>
    /// <param name="q4">Input argument #4</param>
    /// <param name="q5">Input argument #5</param>
    /// <param name="q6">Input argument #6</param>
    ///
    public void robot_arm(Object q1, Object q2, Object q3, Object q4, Object q5, Object 
                    q6)
    {
      mcr.EvaluateFunction(0, "robot_arm", q1, q2, q3, q4, q5, q6);
    }


    /// <summary>
    /// Provides the standard 0-input Object interface to the robot_arm MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ROBOT_ARM Summary of this function goes here
    /// Detailed explanation goes here
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public Object[] robot_arm(int numArgsOut)
    {
      return mcr.EvaluateFunction(numArgsOut, "robot_arm", new Object[]{});
    }


    /// <summary>
    /// Provides the standard 1-input Object interface to the robot_arm MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ROBOT_ARM Summary of this function goes here
    /// Detailed explanation goes here
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="q1">Input argument #1</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public Object[] robot_arm(int numArgsOut, Object q1)
    {
      return mcr.EvaluateFunction(numArgsOut, "robot_arm", q1);
    }


    /// <summary>
    /// Provides the standard 2-input Object interface to the robot_arm MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ROBOT_ARM Summary of this function goes here
    /// Detailed explanation goes here
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="q1">Input argument #1</param>
    /// <param name="q2">Input argument #2</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public Object[] robot_arm(int numArgsOut, Object q1, Object q2)
    {
      return mcr.EvaluateFunction(numArgsOut, "robot_arm", q1, q2);
    }


    /// <summary>
    /// Provides the standard 3-input Object interface to the robot_arm MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ROBOT_ARM Summary of this function goes here
    /// Detailed explanation goes here
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="q1">Input argument #1</param>
    /// <param name="q2">Input argument #2</param>
    /// <param name="q3">Input argument #3</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public Object[] robot_arm(int numArgsOut, Object q1, Object q2, Object q3)
    {
      return mcr.EvaluateFunction(numArgsOut, "robot_arm", q1, q2, q3);
    }


    /// <summary>
    /// Provides the standard 4-input Object interface to the robot_arm MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ROBOT_ARM Summary of this function goes here
    /// Detailed explanation goes here
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="q1">Input argument #1</param>
    /// <param name="q2">Input argument #2</param>
    /// <param name="q3">Input argument #3</param>
    /// <param name="q4">Input argument #4</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public Object[] robot_arm(int numArgsOut, Object q1, Object q2, Object q3, Object q4)
    {
      return mcr.EvaluateFunction(numArgsOut, "robot_arm", q1, q2, q3, q4);
    }


    /// <summary>
    /// Provides the standard 5-input Object interface to the robot_arm MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ROBOT_ARM Summary of this function goes here
    /// Detailed explanation goes here
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
    public Object[] robot_arm(int numArgsOut, Object q1, Object q2, Object q3, Object q4, 
                        Object q5)
    {
      return mcr.EvaluateFunction(numArgsOut, "robot_arm", q1, q2, q3, q4, q5);
    }


    /// <summary>
    /// Provides the standard 6-input Object interface to the robot_arm MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ROBOT_ARM Summary of this function goes here
    /// Detailed explanation goes here
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
    public Object[] robot_arm(int numArgsOut, Object q1, Object q2, Object q3, Object q4, 
                        Object q5, Object q6)
    {
      return mcr.EvaluateFunction(numArgsOut, "robot_arm", q1, q2, q3, q4, q5, q6);
    }


    /// <summary>
    /// Provides an interface for the robot_arm function in which the input and output
    /// arguments are specified as an array of Objects.
    /// </summary>
    /// <remarks>
    /// This method will allocate and return by reference the output argument
    /// array.<newpara></newpara>
    /// M-Documentation:
    /// ROBOT_ARM Summary of this function goes here
    /// Detailed explanation goes here
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return</param>
    /// <param name= "argsOut">Array of Object output arguments</param>
    /// <param name= "argsIn">Array of Object input arguments</param>
    /// <param name= "varArgsIn">Array of Object representing variable input
    /// arguments</param>
    ///
    [MATLABSignature("robot_arm", 6, 0, 0)]
    protected void robot_arm(int numArgsOut, ref Object[] argsOut, Object[] argsIn, params Object[] varArgsIn)
    {
        mcr.EvaluateFunctionForTypeSafeCall("robot_arm", numArgsOut, ref argsOut, argsIn, varArgsIn);
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
