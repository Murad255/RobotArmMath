/*
* MATLAB Compiler: 7.0 (R2018b)
* Date: Tue May 18 22:36:53 2021
* Arguments:
* "-B""macro_default""-W""dotnet:kinematicMArm,KinematicMArmTools,4.0,private""-T""link:li
* b""-d""G:\GoogleDisck\_online\SIENCE\Simulation_matlab\export\kinematicMArm\for_tes
* ting""-v""class{KinematicMArmTools:G:\GoogleDisck\_online\SIENCE\Simulation_matlab\
* export\begin.m,G:\GoogleDisck\_online\SIENCE\Simulation_matlab\export\CalculateI.m,
* G:\GoogleDisck\_online\SIENCE\Simulation_matlab\export\forwardKinematics.m,G:\Googl
* eDisck\_online\SIENCE\Simulation_matlab\export\inverceKinematic.m,G:\GoogleDisck\
* _online\SIENCE\Simulation_matlab\export\inverceKinematicI.m,G:\GoogleDisck\_onli
* ne\SIENCE\Simulation_matlab\export\robot_arm_I.m,G:\GoogleDisck\_online\SIENCE\Simu
* lation_matlab\export\robot_arm_lin.m}"
*/
using System;
using System.Reflection;
using System.IO;
using MathWorks.MATLAB.NET.Arrays;
using MathWorks.MATLAB.NET.Utility;

#if SHARED
[assembly: System.Reflection.AssemblyKeyFile(@"")]
#endif

namespace kinematicMArmNative
{

  /// <summary>
  /// The KinematicMArmTools class provides a CLS compliant, Object (native) interface to
  /// the MATLAB functions contained in the files:
  /// <newpara></newpara>
  /// G:\GoogleDisck\учёба_online\SIENCE\Simulation_matlab\export\begin.m
  /// <newpara></newpara>
  /// G:\GoogleDisck\учёба_online\SIENCE\Simulation_matlab\export\CalculateI.m
  /// <newpara></newpara>
  /// G:\GoogleDisck\учёба_online\SIENCE\Simulation_matlab\export\forwardKinematics.m
  /// <newpara></newpara>
  /// G:\GoogleDisck\учёба_online\SIENCE\Simulation_matlab\export\inverceKinematic.m
  /// <newpara></newpara>
  /// G:\GoogleDisck\учёба_online\SIENCE\Simulation_matlab\export\inverceKinematicI.m
  /// <newpara></newpara>
  /// G:\GoogleDisck\учёба_online\SIENCE\Simulation_matlab\export\robot_arm_I.m
  /// <newpara></newpara>
  /// G:\GoogleDisck\учёба_online\SIENCE\Simulation_matlab\export\robot_arm_lin.m
  /// </summary>
  /// <remarks>
  /// @Version 4.0
  /// </remarks>
  public class KinematicMArmTools : IDisposable
  {
    #region Constructors

    /// <summary internal= "true">
    /// The static constructor instantiates and initializes the MATLAB Runtime instance.
    /// </summary>
    static KinematicMArmTools()
    {
      if (MWMCR.MCRAppInitialized)
      {
        try
        {
          Assembly assembly= Assembly.GetExecutingAssembly();

          string ctfFilePath= assembly.Location;

          int lastDelimiter= ctfFilePath.LastIndexOf(@"\");

          ctfFilePath= ctfFilePath.Remove(lastDelimiter, (ctfFilePath.Length - lastDelimiter));

          string ctfFileName = "kinematicMArm.ctf";

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
    /// Constructs a new instance of the KinematicMArmTools class.
    /// </summary>
    public KinematicMArmTools()
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
    ~KinematicMArmTools()
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
    /// Provides a void output, 0-input Objectinterface to the begin MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// axes('Xlim',[-900 900], 'Ylim',[-900 900], 'Zlim',[-500 900]);
    /// </remarks>
    ///
    public void begin()
    {
      mcr.EvaluateFunction(0, "begin", new Object[]{});
    }


    /// <summary>
    /// Provides the standard 0-input Object interface to the begin MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// axes('Xlim',[-900 900], 'Ylim',[-900 900], 'Zlim',[-500 900]);
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public Object[] begin(int numArgsOut)
    {
      return mcr.EvaluateFunction(numArgsOut, "begin", new Object[]{});
    }


    /// <summary>
    /// Provides an interface for the begin function in which the input and output
    /// arguments are specified as an array of Objects.
    /// </summary>
    /// <remarks>
    /// This method will allocate and return by reference the output argument
    /// array.<newpara></newpara>
    /// M-Documentation:
    /// axes('Xlim',[-900 900], 'Ylim',[-900 900], 'Zlim',[-500 900]);
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return</param>
    /// <param name= "argsOut">Array of Object output arguments</param>
    /// <param name= "argsIn">Array of Object input arguments</param>
    /// <param name= "varArgsIn">Array of Object representing variable input
    /// arguments</param>
    ///
    [MATLABSignature("begin", 0, 0, 0)]
    protected void begin(int numArgsOut, ref Object[] argsOut, Object[] argsIn, params Object[] varArgsIn)
    {
        mcr.EvaluateFunctionForTypeSafeCall("begin", numArgsOut, ref argsOut, argsIn, varArgsIn);
    }
    /// <summary>
    /// Provides a single output, 0-input Objectinterface to the CalculateI MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// indicator wrist calculation
    /// </remarks>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object CalculateI()
    {
      return mcr.EvaluateFunction("CalculateI", new Object[]{});
    }


    /// <summary>
    /// Provides a single output, 1-input Objectinterface to the CalculateI MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// indicator wrist calculation
    /// </remarks>
    /// <param name="A1">Input argument #1</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object CalculateI(Object A1)
    {
      return mcr.EvaluateFunction("CalculateI", A1);
    }


    /// <summary>
    /// Provides a single output, 2-input Objectinterface to the CalculateI MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// indicator wrist calculation
    /// </remarks>
    /// <param name="A1">Input argument #1</param>
    /// <param name="A2">Input argument #2</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object CalculateI(Object A1, Object A2)
    {
      return mcr.EvaluateFunction("CalculateI", A1, A2);
    }


    /// <summary>
    /// Provides a single output, 3-input Objectinterface to the CalculateI MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// indicator wrist calculation
    /// </remarks>
    /// <param name="A1">Input argument #1</param>
    /// <param name="A2">Input argument #2</param>
    /// <param name="A3">Input argument #3</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object CalculateI(Object A1, Object A2, Object A3)
    {
      return mcr.EvaluateFunction("CalculateI", A1, A2, A3);
    }


    /// <summary>
    /// Provides a single output, 4-input Objectinterface to the CalculateI MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// indicator wrist calculation
    /// </remarks>
    /// <param name="A1">Input argument #1</param>
    /// <param name="A2">Input argument #2</param>
    /// <param name="A3">Input argument #3</param>
    /// <param name="A4">Input argument #4</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object CalculateI(Object A1, Object A2, Object A3, Object A4)
    {
      return mcr.EvaluateFunction("CalculateI", A1, A2, A3, A4);
    }


    /// <summary>
    /// Provides a single output, 5-input Objectinterface to the CalculateI MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// indicator wrist calculation
    /// </remarks>
    /// <param name="A1">Input argument #1</param>
    /// <param name="A2">Input argument #2</param>
    /// <param name="A3">Input argument #3</param>
    /// <param name="A4">Input argument #4</param>
    /// <param name="A5">Input argument #5</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object CalculateI(Object A1, Object A2, Object A3, Object A4, Object A5)
    {
      return mcr.EvaluateFunction("CalculateI", A1, A2, A3, A4, A5);
    }


    /// <summary>
    /// Provides a single output, 6-input Objectinterface to the CalculateI MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// indicator wrist calculation
    /// </remarks>
    /// <param name="A1">Input argument #1</param>
    /// <param name="A2">Input argument #2</param>
    /// <param name="A3">Input argument #3</param>
    /// <param name="A4">Input argument #4</param>
    /// <param name="A5">Input argument #5</param>
    /// <param name="A6">Input argument #6</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object CalculateI(Object A1, Object A2, Object A3, Object A4, Object A5, 
                       Object A6)
    {
      return mcr.EvaluateFunction("CalculateI", A1, A2, A3, A4, A5, A6);
    }


    /// <summary>
    /// Provides the standard 0-input Object interface to the CalculateI MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// indicator wrist calculation
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public Object[] CalculateI(int numArgsOut)
    {
      return mcr.EvaluateFunction(numArgsOut, "CalculateI", new Object[]{});
    }


    /// <summary>
    /// Provides the standard 1-input Object interface to the CalculateI MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// indicator wrist calculation
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="A1">Input argument #1</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public Object[] CalculateI(int numArgsOut, Object A1)
    {
      return mcr.EvaluateFunction(numArgsOut, "CalculateI", A1);
    }


    /// <summary>
    /// Provides the standard 2-input Object interface to the CalculateI MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// indicator wrist calculation
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="A1">Input argument #1</param>
    /// <param name="A2">Input argument #2</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public Object[] CalculateI(int numArgsOut, Object A1, Object A2)
    {
      return mcr.EvaluateFunction(numArgsOut, "CalculateI", A1, A2);
    }


    /// <summary>
    /// Provides the standard 3-input Object interface to the CalculateI MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// indicator wrist calculation
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="A1">Input argument #1</param>
    /// <param name="A2">Input argument #2</param>
    /// <param name="A3">Input argument #3</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public Object[] CalculateI(int numArgsOut, Object A1, Object A2, Object A3)
    {
      return mcr.EvaluateFunction(numArgsOut, "CalculateI", A1, A2, A3);
    }


    /// <summary>
    /// Provides the standard 4-input Object interface to the CalculateI MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// indicator wrist calculation
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="A1">Input argument #1</param>
    /// <param name="A2">Input argument #2</param>
    /// <param name="A3">Input argument #3</param>
    /// <param name="A4">Input argument #4</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public Object[] CalculateI(int numArgsOut, Object A1, Object A2, Object A3, Object A4)
    {
      return mcr.EvaluateFunction(numArgsOut, "CalculateI", A1, A2, A3, A4);
    }


    /// <summary>
    /// Provides the standard 5-input Object interface to the CalculateI MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// indicator wrist calculation
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="A1">Input argument #1</param>
    /// <param name="A2">Input argument #2</param>
    /// <param name="A3">Input argument #3</param>
    /// <param name="A4">Input argument #4</param>
    /// <param name="A5">Input argument #5</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public Object[] CalculateI(int numArgsOut, Object A1, Object A2, Object A3, Object 
                         A4, Object A5)
    {
      return mcr.EvaluateFunction(numArgsOut, "CalculateI", A1, A2, A3, A4, A5);
    }


    /// <summary>
    /// Provides the standard 6-input Object interface to the CalculateI MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// indicator wrist calculation
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="A1">Input argument #1</param>
    /// <param name="A2">Input argument #2</param>
    /// <param name="A3">Input argument #3</param>
    /// <param name="A4">Input argument #4</param>
    /// <param name="A5">Input argument #5</param>
    /// <param name="A6">Input argument #6</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public Object[] CalculateI(int numArgsOut, Object A1, Object A2, Object A3, Object 
                         A4, Object A5, Object A6)
    {
      return mcr.EvaluateFunction(numArgsOut, "CalculateI", A1, A2, A3, A4, A5, A6);
    }


    /// <summary>
    /// Provides an interface for the CalculateI function in which the input and output
    /// arguments are specified as an array of Objects.
    /// </summary>
    /// <remarks>
    /// This method will allocate and return by reference the output argument
    /// array.<newpara></newpara>
    /// M-Documentation:
    /// indicator wrist calculation
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return</param>
    /// <param name= "argsOut">Array of Object output arguments</param>
    /// <param name= "argsIn">Array of Object input arguments</param>
    /// <param name= "varArgsIn">Array of Object representing variable input
    /// arguments</param>
    ///
    [MATLABSignature("CalculateI", 6, 3, 0)]
    protected void CalculateI(int numArgsOut, ref Object[] argsOut, Object[] argsIn, params Object[] varArgsIn)
    {
        mcr.EvaluateFunctionForTypeSafeCall("CalculateI", numArgsOut, ref argsOut, argsIn, varArgsIn);
    }
    /// <summary>
    /// Provides a single output, 0-input Objectinterface to the forwardKinematics MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ОЗК + изобразить модель робота при showFlag=1, reachFlag =0 в слечае
    /// невозможности достижения точки
    /// </remarks>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object forwardKinematics()
    {
      return mcr.EvaluateFunction("forwardKinematics", new Object[]{});
    }


    /// <summary>
    /// Provides a single output, 1-input Objectinterface to the forwardKinematics MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ОЗК + изобразить модель робота при showFlag=1, reachFlag =0 в слечае
    /// невозможности достижения точки
    /// </remarks>
    /// <param name="A1">Input argument #1</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object forwardKinematics(Object A1)
    {
      return mcr.EvaluateFunction("forwardKinematics", A1);
    }


    /// <summary>
    /// Provides a single output, 2-input Objectinterface to the forwardKinematics MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ОЗК + изобразить модель робота при showFlag=1, reachFlag =0 в слечае
    /// невозможности достижения точки
    /// </remarks>
    /// <param name="A1">Input argument #1</param>
    /// <param name="A2">Input argument #2</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object forwardKinematics(Object A1, Object A2)
    {
      return mcr.EvaluateFunction("forwardKinematics", A1, A2);
    }


    /// <summary>
    /// Provides a single output, 3-input Objectinterface to the forwardKinematics MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ОЗК + изобразить модель робота при showFlag=1, reachFlag =0 в слечае
    /// невозможности достижения точки
    /// </remarks>
    /// <param name="A1">Input argument #1</param>
    /// <param name="A2">Input argument #2</param>
    /// <param name="A3">Input argument #3</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object forwardKinematics(Object A1, Object A2, Object A3)
    {
      return mcr.EvaluateFunction("forwardKinematics", A1, A2, A3);
    }


    /// <summary>
    /// Provides a single output, 4-input Objectinterface to the forwardKinematics MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ОЗК + изобразить модель робота при showFlag=1, reachFlag =0 в слечае
    /// невозможности достижения точки
    /// </remarks>
    /// <param name="A1">Input argument #1</param>
    /// <param name="A2">Input argument #2</param>
    /// <param name="A3">Input argument #3</param>
    /// <param name="A4">Input argument #4</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object forwardKinematics(Object A1, Object A2, Object A3, Object A4)
    {
      return mcr.EvaluateFunction("forwardKinematics", A1, A2, A3, A4);
    }


    /// <summary>
    /// Provides a single output, 5-input Objectinterface to the forwardKinematics MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ОЗК + изобразить модель робота при showFlag=1, reachFlag =0 в слечае
    /// невозможности достижения точки
    /// </remarks>
    /// <param name="A1">Input argument #1</param>
    /// <param name="A2">Input argument #2</param>
    /// <param name="A3">Input argument #3</param>
    /// <param name="A4">Input argument #4</param>
    /// <param name="A5">Input argument #5</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object forwardKinematics(Object A1, Object A2, Object A3, Object A4, Object A5)
    {
      return mcr.EvaluateFunction("forwardKinematics", A1, A2, A3, A4, A5);
    }


    /// <summary>
    /// Provides a single output, 6-input Objectinterface to the forwardKinematics MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ОЗК + изобразить модель робота при showFlag=1, reachFlag =0 в слечае
    /// невозможности достижения точки
    /// </remarks>
    /// <param name="A1">Input argument #1</param>
    /// <param name="A2">Input argument #2</param>
    /// <param name="A3">Input argument #3</param>
    /// <param name="A4">Input argument #4</param>
    /// <param name="A5">Input argument #5</param>
    /// <param name="A6">Input argument #6</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object forwardKinematics(Object A1, Object A2, Object A3, Object A4, Object 
                              A5, Object A6)
    {
      return mcr.EvaluateFunction("forwardKinematics", A1, A2, A3, A4, A5, A6);
    }


    /// <summary>
    /// Provides the standard 0-input Object interface to the forwardKinematics MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ОЗК + изобразить модель робота при showFlag=1, reachFlag =0 в слечае
    /// невозможности достижения точки
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public Object[] forwardKinematics(int numArgsOut)
    {
      return mcr.EvaluateFunction(numArgsOut, "forwardKinematics", new Object[]{});
    }


    /// <summary>
    /// Provides the standard 1-input Object interface to the forwardKinematics MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ОЗК + изобразить модель робота при showFlag=1, reachFlag =0 в слечае
    /// невозможности достижения точки
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="A1">Input argument #1</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public Object[] forwardKinematics(int numArgsOut, Object A1)
    {
      return mcr.EvaluateFunction(numArgsOut, "forwardKinematics", A1);
    }


    /// <summary>
    /// Provides the standard 2-input Object interface to the forwardKinematics MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ОЗК + изобразить модель робота при showFlag=1, reachFlag =0 в слечае
    /// невозможности достижения точки
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="A1">Input argument #1</param>
    /// <param name="A2">Input argument #2</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public Object[] forwardKinematics(int numArgsOut, Object A1, Object A2)
    {
      return mcr.EvaluateFunction(numArgsOut, "forwardKinematics", A1, A2);
    }


    /// <summary>
    /// Provides the standard 3-input Object interface to the forwardKinematics MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ОЗК + изобразить модель робота при showFlag=1, reachFlag =0 в слечае
    /// невозможности достижения точки
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="A1">Input argument #1</param>
    /// <param name="A2">Input argument #2</param>
    /// <param name="A3">Input argument #3</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public Object[] forwardKinematics(int numArgsOut, Object A1, Object A2, Object A3)
    {
      return mcr.EvaluateFunction(numArgsOut, "forwardKinematics", A1, A2, A3);
    }


    /// <summary>
    /// Provides the standard 4-input Object interface to the forwardKinematics MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ОЗК + изобразить модель робота при showFlag=1, reachFlag =0 в слечае
    /// невозможности достижения точки
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="A1">Input argument #1</param>
    /// <param name="A2">Input argument #2</param>
    /// <param name="A3">Input argument #3</param>
    /// <param name="A4">Input argument #4</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public Object[] forwardKinematics(int numArgsOut, Object A1, Object A2, Object A3, 
                                Object A4)
    {
      return mcr.EvaluateFunction(numArgsOut, "forwardKinematics", A1, A2, A3, A4);
    }


    /// <summary>
    /// Provides the standard 5-input Object interface to the forwardKinematics MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ОЗК + изобразить модель робота при showFlag=1, reachFlag =0 в слечае
    /// невозможности достижения точки
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="A1">Input argument #1</param>
    /// <param name="A2">Input argument #2</param>
    /// <param name="A3">Input argument #3</param>
    /// <param name="A4">Input argument #4</param>
    /// <param name="A5">Input argument #5</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public Object[] forwardKinematics(int numArgsOut, Object A1, Object A2, Object A3, 
                                Object A4, Object A5)
    {
      return mcr.EvaluateFunction(numArgsOut, "forwardKinematics", A1, A2, A3, A4, A5);
    }


    /// <summary>
    /// Provides the standard 6-input Object interface to the forwardKinematics MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ОЗК + изобразить модель робота при showFlag=1, reachFlag =0 в слечае
    /// невозможности достижения точки
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="A1">Input argument #1</param>
    /// <param name="A2">Input argument #2</param>
    /// <param name="A3">Input argument #3</param>
    /// <param name="A4">Input argument #4</param>
    /// <param name="A5">Input argument #5</param>
    /// <param name="A6">Input argument #6</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public Object[] forwardKinematics(int numArgsOut, Object A1, Object A2, Object A3, 
                                Object A4, Object A5, Object A6)
    {
      return mcr.EvaluateFunction(numArgsOut, "forwardKinematics", A1, A2, A3, A4, A5, A6);
    }


    /// <summary>
    /// Provides an interface for the forwardKinematics function in which the input and
    /// output
    /// arguments are specified as an array of Objects.
    /// </summary>
    /// <remarks>
    /// This method will allocate and return by reference the output argument
    /// array.<newpara></newpara>
    /// M-Documentation:
    /// ОЗК + изобразить модель робота при showFlag=1, reachFlag =0 в слечае
    /// невозможности достижения точки
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return</param>
    /// <param name= "argsOut">Array of Object output arguments</param>
    /// <param name= "argsIn">Array of Object input arguments</param>
    /// <param name= "varArgsIn">Array of Object representing variable input
    /// arguments</param>
    ///
    [MATLABSignature("forwardKinematics", 6, 4, 0)]
    protected void forwardKinematics(int numArgsOut, ref Object[] argsOut, Object[] argsIn, params Object[] varArgsIn)
    {
        mcr.EvaluateFunctionForTypeSafeCall("forwardKinematics", numArgsOut, ref argsOut, argsIn, varArgsIn);
    }
    /// <summary>
    /// Provides a single output, 0-input Objectinterface to the inverceKinematic MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// d=[0;150;0;430;0;60];
    /// a=[0;430;-20;0;0;0];
    /// </remarks>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object inverceKinematic()
    {
      return mcr.EvaluateFunction("inverceKinematic", new Object[]{});
    }


    /// <summary>
    /// Provides a single output, 1-input Objectinterface to the inverceKinematic MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// d=[0;150;0;430;0;60];
    /// a=[0;430;-20;0;0;0];
    /// </remarks>
    /// <param name="x">Input argument #1</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object inverceKinematic(Object x)
    {
      return mcr.EvaluateFunction("inverceKinematic", x);
    }


    /// <summary>
    /// Provides a single output, 2-input Objectinterface to the inverceKinematic MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// d=[0;150;0;430;0;60];
    /// a=[0;430;-20;0;0;0];
    /// </remarks>
    /// <param name="x">Input argument #1</param>
    /// <param name="y">Input argument #2</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object inverceKinematic(Object x, Object y)
    {
      return mcr.EvaluateFunction("inverceKinematic", x, y);
    }


    /// <summary>
    /// Provides a single output, 3-input Objectinterface to the inverceKinematic MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// d=[0;150;0;430;0;60];
    /// a=[0;430;-20;0;0;0];
    /// </remarks>
    /// <param name="x">Input argument #1</param>
    /// <param name="y">Input argument #2</param>
    /// <param name="z">Input argument #3</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object inverceKinematic(Object x, Object y, Object z)
    {
      return mcr.EvaluateFunction("inverceKinematic", x, y, z);
    }


    /// <summary>
    /// Provides a single output, 4-input Objectinterface to the inverceKinematic MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// d=[0;150;0;430;0;60];
    /// a=[0;430;-20;0;0;0];
    /// </remarks>
    /// <param name="x">Input argument #1</param>
    /// <param name="y">Input argument #2</param>
    /// <param name="z">Input argument #3</param>
    /// <param name="A">Input argument #4</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object inverceKinematic(Object x, Object y, Object z, Object A)
    {
      return mcr.EvaluateFunction("inverceKinematic", x, y, z, A);
    }


    /// <summary>
    /// Provides the standard 0-input Object interface to the inverceKinematic MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// d=[0;150;0;430;0;60];
    /// a=[0;430;-20;0;0;0];
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public Object[] inverceKinematic(int numArgsOut)
    {
      return mcr.EvaluateFunction(numArgsOut, "inverceKinematic", new Object[]{});
    }


    /// <summary>
    /// Provides the standard 1-input Object interface to the inverceKinematic MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// d=[0;150;0;430;0;60];
    /// a=[0;430;-20;0;0;0];
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="x">Input argument #1</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public Object[] inverceKinematic(int numArgsOut, Object x)
    {
      return mcr.EvaluateFunction(numArgsOut, "inverceKinematic", x);
    }


    /// <summary>
    /// Provides the standard 2-input Object interface to the inverceKinematic MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// d=[0;150;0;430;0;60];
    /// a=[0;430;-20;0;0;0];
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="x">Input argument #1</param>
    /// <param name="y">Input argument #2</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public Object[] inverceKinematic(int numArgsOut, Object x, Object y)
    {
      return mcr.EvaluateFunction(numArgsOut, "inverceKinematic", x, y);
    }


    /// <summary>
    /// Provides the standard 3-input Object interface to the inverceKinematic MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// d=[0;150;0;430;0;60];
    /// a=[0;430;-20;0;0;0];
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="x">Input argument #1</param>
    /// <param name="y">Input argument #2</param>
    /// <param name="z">Input argument #3</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public Object[] inverceKinematic(int numArgsOut, Object x, Object y, Object z)
    {
      return mcr.EvaluateFunction(numArgsOut, "inverceKinematic", x, y, z);
    }


    /// <summary>
    /// Provides the standard 4-input Object interface to the inverceKinematic MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// d=[0;150;0;430;0;60];
    /// a=[0;430;-20;0;0;0];
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="x">Input argument #1</param>
    /// <param name="y">Input argument #2</param>
    /// <param name="z">Input argument #3</param>
    /// <param name="A">Input argument #4</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public Object[] inverceKinematic(int numArgsOut, Object x, Object y, Object z, Object 
                               A)
    {
      return mcr.EvaluateFunction(numArgsOut, "inverceKinematic", x, y, z, A);
    }


    /// <summary>
    /// Provides an interface for the inverceKinematic function in which the input and
    /// output
    /// arguments are specified as an array of Objects.
    /// </summary>
    /// <remarks>
    /// This method will allocate and return by reference the output argument
    /// array.<newpara></newpara>
    /// M-Documentation:
    /// d=[0;150;0;430;0;60];
    /// a=[0;430;-20;0;0;0];
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return</param>
    /// <param name= "argsOut">Array of Object output arguments</param>
    /// <param name= "argsIn">Array of Object input arguments</param>
    /// <param name= "varArgsIn">Array of Object representing variable input
    /// arguments</param>
    ///
    [MATLABSignature("inverceKinematic", 4, 6, 0)]
    protected void inverceKinematic(int numArgsOut, ref Object[] argsOut, Object[] argsIn, params Object[] varArgsIn)
    {
        mcr.EvaluateFunctionForTypeSafeCall("inverceKinematic", numArgsOut, ref argsOut, argsIn, varArgsIn);
    }
    /// <summary>
    /// Provides a single output, 0-input Objectinterface to the inverceKinematicI MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// d=[0;150;0;430;0;60];
    /// a=[0;430;-20;0;0;0];
    /// </remarks>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object inverceKinematicI()
    {
      return mcr.EvaluateFunction("inverceKinematicI", new Object[]{});
    }


    /// <summary>
    /// Provides a single output, 1-input Objectinterface to the inverceKinematicI MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// d=[0;150;0;430;0;60];
    /// a=[0;430;-20;0;0;0];
    /// </remarks>
    /// <param name="x">Input argument #1</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object inverceKinematicI(Object x)
    {
      return mcr.EvaluateFunction("inverceKinematicI", x);
    }


    /// <summary>
    /// Provides a single output, 2-input Objectinterface to the inverceKinematicI MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// d=[0;150;0;430;0;60];
    /// a=[0;430;-20;0;0;0];
    /// </remarks>
    /// <param name="x">Input argument #1</param>
    /// <param name="y">Input argument #2</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object inverceKinematicI(Object x, Object y)
    {
      return mcr.EvaluateFunction("inverceKinematicI", x, y);
    }


    /// <summary>
    /// Provides a single output, 3-input Objectinterface to the inverceKinematicI MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// d=[0;150;0;430;0;60];
    /// a=[0;430;-20;0;0;0];
    /// </remarks>
    /// <param name="x">Input argument #1</param>
    /// <param name="y">Input argument #2</param>
    /// <param name="z">Input argument #3</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object inverceKinematicI(Object x, Object y, Object z)
    {
      return mcr.EvaluateFunction("inverceKinematicI", x, y, z);
    }


    /// <summary>
    /// Provides a single output, 4-input Objectinterface to the inverceKinematicI MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// d=[0;150;0;430;0;60];
    /// a=[0;430;-20;0;0;0];
    /// </remarks>
    /// <param name="x">Input argument #1</param>
    /// <param name="y">Input argument #2</param>
    /// <param name="z">Input argument #3</param>
    /// <param name="A">Input argument #4</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object inverceKinematicI(Object x, Object y, Object z, Object A)
    {
      return mcr.EvaluateFunction("inverceKinematicI", x, y, z, A);
    }


    /// <summary>
    /// Provides a single output, 5-input Objectinterface to the inverceKinematicI MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// d=[0;150;0;430;0;60];
    /// a=[0;430;-20;0;0;0];
    /// </remarks>
    /// <param name="x">Input argument #1</param>
    /// <param name="y">Input argument #2</param>
    /// <param name="z">Input argument #3</param>
    /// <param name="A">Input argument #4</param>
    /// <param name="sign_hand">Input argument #5</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object inverceKinematicI(Object x, Object y, Object z, Object A, Object 
                              sign_hand)
    {
      return mcr.EvaluateFunction("inverceKinematicI", x, y, z, A, sign_hand);
    }


    /// <summary>
    /// Provides a single output, 6-input Objectinterface to the inverceKinematicI MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// d=[0;150;0;430;0;60];
    /// a=[0;430;-20;0;0;0];
    /// </remarks>
    /// <param name="x">Input argument #1</param>
    /// <param name="y">Input argument #2</param>
    /// <param name="z">Input argument #3</param>
    /// <param name="A">Input argument #4</param>
    /// <param name="sign_hand">Input argument #5</param>
    /// <param name="sign_elbow">Input argument #6</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object inverceKinematicI(Object x, Object y, Object z, Object A, Object 
                              sign_hand, Object sign_elbow)
    {
      return mcr.EvaluateFunction("inverceKinematicI", x, y, z, A, sign_hand, sign_elbow);
    }


    /// <summary>
    /// Provides a single output, 7-input Objectinterface to the inverceKinematicI MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// d=[0;150;0;430;0;60];
    /// a=[0;430;-20;0;0;0];
    /// </remarks>
    /// <param name="x">Input argument #1</param>
    /// <param name="y">Input argument #2</param>
    /// <param name="z">Input argument #3</param>
    /// <param name="A">Input argument #4</param>
    /// <param name="sign_hand">Input argument #5</param>
    /// <param name="sign_elbow">Input argument #6</param>
    /// <param name="or_ind">Input argument #7</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object inverceKinematicI(Object x, Object y, Object z, Object A, Object 
                              sign_hand, Object sign_elbow, Object or_ind)
    {
      return mcr.EvaluateFunction("inverceKinematicI", x, y, z, A, sign_hand, sign_elbow, or_ind);
    }


    /// <summary>
    /// Provides the standard 0-input Object interface to the inverceKinematicI MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// d=[0;150;0;430;0;60];
    /// a=[0;430;-20;0;0;0];
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public Object[] inverceKinematicI(int numArgsOut)
    {
      return mcr.EvaluateFunction(numArgsOut, "inverceKinematicI", new Object[]{});
    }


    /// <summary>
    /// Provides the standard 1-input Object interface to the inverceKinematicI MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// d=[0;150;0;430;0;60];
    /// a=[0;430;-20;0;0;0];
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="x">Input argument #1</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public Object[] inverceKinematicI(int numArgsOut, Object x)
    {
      return mcr.EvaluateFunction(numArgsOut, "inverceKinematicI", x);
    }


    /// <summary>
    /// Provides the standard 2-input Object interface to the inverceKinematicI MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// d=[0;150;0;430;0;60];
    /// a=[0;430;-20;0;0;0];
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="x">Input argument #1</param>
    /// <param name="y">Input argument #2</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public Object[] inverceKinematicI(int numArgsOut, Object x, Object y)
    {
      return mcr.EvaluateFunction(numArgsOut, "inverceKinematicI", x, y);
    }


    /// <summary>
    /// Provides the standard 3-input Object interface to the inverceKinematicI MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// d=[0;150;0;430;0;60];
    /// a=[0;430;-20;0;0;0];
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="x">Input argument #1</param>
    /// <param name="y">Input argument #2</param>
    /// <param name="z">Input argument #3</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public Object[] inverceKinematicI(int numArgsOut, Object x, Object y, Object z)
    {
      return mcr.EvaluateFunction(numArgsOut, "inverceKinematicI", x, y, z);
    }


    /// <summary>
    /// Provides the standard 4-input Object interface to the inverceKinematicI MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// d=[0;150;0;430;0;60];
    /// a=[0;430;-20;0;0;0];
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="x">Input argument #1</param>
    /// <param name="y">Input argument #2</param>
    /// <param name="z">Input argument #3</param>
    /// <param name="A">Input argument #4</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public Object[] inverceKinematicI(int numArgsOut, Object x, Object y, Object z, 
                                Object A)
    {
      return mcr.EvaluateFunction(numArgsOut, "inverceKinematicI", x, y, z, A);
    }


    /// <summary>
    /// Provides the standard 5-input Object interface to the inverceKinematicI MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// d=[0;150;0;430;0;60];
    /// a=[0;430;-20;0;0;0];
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="x">Input argument #1</param>
    /// <param name="y">Input argument #2</param>
    /// <param name="z">Input argument #3</param>
    /// <param name="A">Input argument #4</param>
    /// <param name="sign_hand">Input argument #5</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public Object[] inverceKinematicI(int numArgsOut, Object x, Object y, Object z, 
                                Object A, Object sign_hand)
    {
      return mcr.EvaluateFunction(numArgsOut, "inverceKinematicI", x, y, z, A, sign_hand);
    }


    /// <summary>
    /// Provides the standard 6-input Object interface to the inverceKinematicI MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// d=[0;150;0;430;0;60];
    /// a=[0;430;-20;0;0;0];
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="x">Input argument #1</param>
    /// <param name="y">Input argument #2</param>
    /// <param name="z">Input argument #3</param>
    /// <param name="A">Input argument #4</param>
    /// <param name="sign_hand">Input argument #5</param>
    /// <param name="sign_elbow">Input argument #6</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public Object[] inverceKinematicI(int numArgsOut, Object x, Object y, Object z, 
                                Object A, Object sign_hand, Object sign_elbow)
    {
      return mcr.EvaluateFunction(numArgsOut, "inverceKinematicI", x, y, z, A, sign_hand, sign_elbow);
    }


    /// <summary>
    /// Provides the standard 7-input Object interface to the inverceKinematicI MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// d=[0;150;0;430;0;60];
    /// a=[0;430;-20;0;0;0];
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="x">Input argument #1</param>
    /// <param name="y">Input argument #2</param>
    /// <param name="z">Input argument #3</param>
    /// <param name="A">Input argument #4</param>
    /// <param name="sign_hand">Input argument #5</param>
    /// <param name="sign_elbow">Input argument #6</param>
    /// <param name="or_ind">Input argument #7</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public Object[] inverceKinematicI(int numArgsOut, Object x, Object y, Object z, 
                                Object A, Object sign_hand, Object sign_elbow, Object 
                                or_ind)
    {
      return mcr.EvaluateFunction(numArgsOut, "inverceKinematicI", x, y, z, A, sign_hand, sign_elbow, or_ind);
    }


    /// <summary>
    /// Provides an interface for the inverceKinematicI function in which the input and
    /// output
    /// arguments are specified as an array of Objects.
    /// </summary>
    /// <remarks>
    /// This method will allocate and return by reference the output argument
    /// array.<newpara></newpara>
    /// M-Documentation:
    /// d=[0;150;0;430;0;60];
    /// a=[0;430;-20;0;0;0];
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return</param>
    /// <param name= "argsOut">Array of Object output arguments</param>
    /// <param name= "argsIn">Array of Object input arguments</param>
    /// <param name= "varArgsIn">Array of Object representing variable input
    /// arguments</param>
    ///
    [MATLABSignature("inverceKinematicI", 7, 6, 0)]
    protected void inverceKinematicI(int numArgsOut, ref Object[] argsOut, Object[] argsIn, params Object[] varArgsIn)
    {
        mcr.EvaluateFunctionForTypeSafeCall("inverceKinematicI", numArgsOut, ref argsOut, argsIn, varArgsIn);
    }
    /// <summary>
    /// Provides a single output, 0-input Objectinterface to the robot_arm_I MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ПЗК+изобразить модель робота
    /// </remarks>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object robot_arm_I()
    {
      return mcr.EvaluateFunction("robot_arm_I", new Object[]{});
    }


    /// <summary>
    /// Provides a single output, 1-input Objectinterface to the robot_arm_I MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ПЗК+изобразить модель робота
    /// </remarks>
    /// <param name="A1">Input argument #1</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object robot_arm_I(Object A1)
    {
      return mcr.EvaluateFunction("robot_arm_I", A1);
    }


    /// <summary>
    /// Provides a single output, 2-input Objectinterface to the robot_arm_I MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ПЗК+изобразить модель робота
    /// </remarks>
    /// <param name="A1">Input argument #1</param>
    /// <param name="A2">Input argument #2</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object robot_arm_I(Object A1, Object A2)
    {
      return mcr.EvaluateFunction("robot_arm_I", A1, A2);
    }


    /// <summary>
    /// Provides a single output, 3-input Objectinterface to the robot_arm_I MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ПЗК+изобразить модель робота
    /// </remarks>
    /// <param name="A1">Input argument #1</param>
    /// <param name="A2">Input argument #2</param>
    /// <param name="A3">Input argument #3</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object robot_arm_I(Object A1, Object A2, Object A3)
    {
      return mcr.EvaluateFunction("robot_arm_I", A1, A2, A3);
    }


    /// <summary>
    /// Provides a single output, 4-input Objectinterface to the robot_arm_I MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ПЗК+изобразить модель робота
    /// </remarks>
    /// <param name="A1">Input argument #1</param>
    /// <param name="A2">Input argument #2</param>
    /// <param name="A3">Input argument #3</param>
    /// <param name="A4">Input argument #4</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object robot_arm_I(Object A1, Object A2, Object A3, Object A4)
    {
      return mcr.EvaluateFunction("robot_arm_I", A1, A2, A3, A4);
    }


    /// <summary>
    /// Provides a single output, 5-input Objectinterface to the robot_arm_I MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ПЗК+изобразить модель робота
    /// </remarks>
    /// <param name="A1">Input argument #1</param>
    /// <param name="A2">Input argument #2</param>
    /// <param name="A3">Input argument #3</param>
    /// <param name="A4">Input argument #4</param>
    /// <param name="A5">Input argument #5</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object robot_arm_I(Object A1, Object A2, Object A3, Object A4, Object A5)
    {
      return mcr.EvaluateFunction("robot_arm_I", A1, A2, A3, A4, A5);
    }


    /// <summary>
    /// Provides a single output, 6-input Objectinterface to the robot_arm_I MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ПЗК+изобразить модель робота
    /// </remarks>
    /// <param name="A1">Input argument #1</param>
    /// <param name="A2">Input argument #2</param>
    /// <param name="A3">Input argument #3</param>
    /// <param name="A4">Input argument #4</param>
    /// <param name="A5">Input argument #5</param>
    /// <param name="A6">Input argument #6</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object robot_arm_I(Object A1, Object A2, Object A3, Object A4, Object A5, 
                        Object A6)
    {
      return mcr.EvaluateFunction("robot_arm_I", A1, A2, A3, A4, A5, A6);
    }


    /// <summary>
    /// Provides a single output, 7-input Objectinterface to the robot_arm_I MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ПЗК+изобразить модель робота
    /// </remarks>
    /// <param name="A1">Input argument #1</param>
    /// <param name="A2">Input argument #2</param>
    /// <param name="A3">Input argument #3</param>
    /// <param name="A4">Input argument #4</param>
    /// <param name="A5">Input argument #5</param>
    /// <param name="A6">Input argument #6</param>
    /// <param name="showFlag">Input argument #7</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object robot_arm_I(Object A1, Object A2, Object A3, Object A4, Object A5, 
                        Object A6, Object showFlag)
    {
      return mcr.EvaluateFunction("robot_arm_I", A1, A2, A3, A4, A5, A6, showFlag);
    }


    /// <summary>
    /// Provides the standard 0-input Object interface to the robot_arm_I MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ПЗК+изобразить модель робота
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public Object[] robot_arm_I(int numArgsOut)
    {
      return mcr.EvaluateFunction(numArgsOut, "robot_arm_I", new Object[]{});
    }


    /// <summary>
    /// Provides the standard 1-input Object interface to the robot_arm_I MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ПЗК+изобразить модель робота
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="A1">Input argument #1</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public Object[] robot_arm_I(int numArgsOut, Object A1)
    {
      return mcr.EvaluateFunction(numArgsOut, "robot_arm_I", A1);
    }


    /// <summary>
    /// Provides the standard 2-input Object interface to the robot_arm_I MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ПЗК+изобразить модель робота
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="A1">Input argument #1</param>
    /// <param name="A2">Input argument #2</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public Object[] robot_arm_I(int numArgsOut, Object A1, Object A2)
    {
      return mcr.EvaluateFunction(numArgsOut, "robot_arm_I", A1, A2);
    }


    /// <summary>
    /// Provides the standard 3-input Object interface to the robot_arm_I MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ПЗК+изобразить модель робота
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="A1">Input argument #1</param>
    /// <param name="A2">Input argument #2</param>
    /// <param name="A3">Input argument #3</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public Object[] robot_arm_I(int numArgsOut, Object A1, Object A2, Object A3)
    {
      return mcr.EvaluateFunction(numArgsOut, "robot_arm_I", A1, A2, A3);
    }


    /// <summary>
    /// Provides the standard 4-input Object interface to the robot_arm_I MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ПЗК+изобразить модель робота
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="A1">Input argument #1</param>
    /// <param name="A2">Input argument #2</param>
    /// <param name="A3">Input argument #3</param>
    /// <param name="A4">Input argument #4</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public Object[] robot_arm_I(int numArgsOut, Object A1, Object A2, Object A3, Object 
                          A4)
    {
      return mcr.EvaluateFunction(numArgsOut, "robot_arm_I", A1, A2, A3, A4);
    }


    /// <summary>
    /// Provides the standard 5-input Object interface to the robot_arm_I MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ПЗК+изобразить модель робота
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="A1">Input argument #1</param>
    /// <param name="A2">Input argument #2</param>
    /// <param name="A3">Input argument #3</param>
    /// <param name="A4">Input argument #4</param>
    /// <param name="A5">Input argument #5</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public Object[] robot_arm_I(int numArgsOut, Object A1, Object A2, Object A3, Object 
                          A4, Object A5)
    {
      return mcr.EvaluateFunction(numArgsOut, "robot_arm_I", A1, A2, A3, A4, A5);
    }


    /// <summary>
    /// Provides the standard 6-input Object interface to the robot_arm_I MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ПЗК+изобразить модель робота
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="A1">Input argument #1</param>
    /// <param name="A2">Input argument #2</param>
    /// <param name="A3">Input argument #3</param>
    /// <param name="A4">Input argument #4</param>
    /// <param name="A5">Input argument #5</param>
    /// <param name="A6">Input argument #6</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public Object[] robot_arm_I(int numArgsOut, Object A1, Object A2, Object A3, Object 
                          A4, Object A5, Object A6)
    {
      return mcr.EvaluateFunction(numArgsOut, "robot_arm_I", A1, A2, A3, A4, A5, A6);
    }


    /// <summary>
    /// Provides the standard 7-input Object interface to the robot_arm_I MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ПЗК+изобразить модель робота
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="A1">Input argument #1</param>
    /// <param name="A2">Input argument #2</param>
    /// <param name="A3">Input argument #3</param>
    /// <param name="A4">Input argument #4</param>
    /// <param name="A5">Input argument #5</param>
    /// <param name="A6">Input argument #6</param>
    /// <param name="showFlag">Input argument #7</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public Object[] robot_arm_I(int numArgsOut, Object A1, Object A2, Object A3, Object 
                          A4, Object A5, Object A6, Object showFlag)
    {
      return mcr.EvaluateFunction(numArgsOut, "robot_arm_I", A1, A2, A3, A4, A5, A6, showFlag);
    }


    /// <summary>
    /// Provides an interface for the robot_arm_I function in which the input and output
    /// arguments are specified as an array of Objects.
    /// </summary>
    /// <remarks>
    /// This method will allocate and return by reference the output argument
    /// array.<newpara></newpara>
    /// M-Documentation:
    /// ПЗК+изобразить модель робота
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return</param>
    /// <param name= "argsOut">Array of Object output arguments</param>
    /// <param name= "argsIn">Array of Object input arguments</param>
    /// <param name= "varArgsIn">Array of Object representing variable input
    /// arguments</param>
    ///
    [MATLABSignature("robot_arm_I", 7, 7, 0)]
    protected void robot_arm_I(int numArgsOut, ref Object[] argsOut, Object[] argsIn, params Object[] varArgsIn)
    {
        mcr.EvaluateFunctionForTypeSafeCall("robot_arm_I", numArgsOut, ref argsOut, argsIn, varArgsIn);
    }
    /// <summary>
    /// Provides a single output, 0-input Objectinterface to the robot_arm_lin MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ПЗК+изобразить модель робота
    /// </remarks>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object robot_arm_lin()
    {
      return mcr.EvaluateFunction("robot_arm_lin", new Object[]{});
    }


    /// <summary>
    /// Provides a single output, 1-input Objectinterface to the robot_arm_lin MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ПЗК+изобразить модель робота
    /// </remarks>
    /// <param name="x">Input argument #1</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object robot_arm_lin(Object x)
    {
      return mcr.EvaluateFunction("robot_arm_lin", x);
    }


    /// <summary>
    /// Provides a single output, 2-input Objectinterface to the robot_arm_lin MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ПЗК+изобразить модель робота
    /// </remarks>
    /// <param name="x">Input argument #1</param>
    /// <param name="y">Input argument #2</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object robot_arm_lin(Object x, Object y)
    {
      return mcr.EvaluateFunction("robot_arm_lin", x, y);
    }


    /// <summary>
    /// Provides a single output, 3-input Objectinterface to the robot_arm_lin MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ПЗК+изобразить модель робота
    /// </remarks>
    /// <param name="x">Input argument #1</param>
    /// <param name="y">Input argument #2</param>
    /// <param name="z">Input argument #3</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object robot_arm_lin(Object x, Object y, Object z)
    {
      return mcr.EvaluateFunction("robot_arm_lin", x, y, z);
    }


    /// <summary>
    /// Provides a single output, 4-input Objectinterface to the robot_arm_lin MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ПЗК+изобразить модель робота
    /// </remarks>
    /// <param name="x">Input argument #1</param>
    /// <param name="y">Input argument #2</param>
    /// <param name="z">Input argument #3</param>
    /// <param name="A">Input argument #4</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object robot_arm_lin(Object x, Object y, Object z, Object A)
    {
      return mcr.EvaluateFunction("robot_arm_lin", x, y, z, A);
    }


    /// <summary>
    /// Provides a single output, 5-input Objectinterface to the robot_arm_lin MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ПЗК+изобразить модель робота
    /// </remarks>
    /// <param name="x">Input argument #1</param>
    /// <param name="y">Input argument #2</param>
    /// <param name="z">Input argument #3</param>
    /// <param name="A">Input argument #4</param>
    /// <param name="i1">Input argument #5</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object robot_arm_lin(Object x, Object y, Object z, Object A, Object i1)
    {
      return mcr.EvaluateFunction("robot_arm_lin", x, y, z, A, i1);
    }


    /// <summary>
    /// Provides a single output, 6-input Objectinterface to the robot_arm_lin MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ПЗК+изобразить модель робота
    /// </remarks>
    /// <param name="x">Input argument #1</param>
    /// <param name="y">Input argument #2</param>
    /// <param name="z">Input argument #3</param>
    /// <param name="A">Input argument #4</param>
    /// <param name="i1">Input argument #5</param>
    /// <param name="i2">Input argument #6</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object robot_arm_lin(Object x, Object y, Object z, Object A, Object i1, Object 
                          i2)
    {
      return mcr.EvaluateFunction("robot_arm_lin", x, y, z, A, i1, i2);
    }


    /// <summary>
    /// Provides a single output, 7-input Objectinterface to the robot_arm_lin MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ПЗК+изобразить модель робота
    /// </remarks>
    /// <param name="x">Input argument #1</param>
    /// <param name="y">Input argument #2</param>
    /// <param name="z">Input argument #3</param>
    /// <param name="A">Input argument #4</param>
    /// <param name="i1">Input argument #5</param>
    /// <param name="i2">Input argument #6</param>
    /// <param name="i3">Input argument #7</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object robot_arm_lin(Object x, Object y, Object z, Object A, Object i1, Object 
                          i2, Object i3)
    {
      return mcr.EvaluateFunction("robot_arm_lin", x, y, z, A, i1, i2, i3);
    }


    /// <summary>
    /// Provides a single output, 8-input Objectinterface to the robot_arm_lin MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ПЗК+изобразить модель робота
    /// </remarks>
    /// <param name="x">Input argument #1</param>
    /// <param name="y">Input argument #2</param>
    /// <param name="z">Input argument #3</param>
    /// <param name="A">Input argument #4</param>
    /// <param name="i1">Input argument #5</param>
    /// <param name="i2">Input argument #6</param>
    /// <param name="i3">Input argument #7</param>
    /// <param name="showFlag">Input argument #8</param>
    /// <returns>An Object containing the first output argument.</returns>
    ///
    public Object robot_arm_lin(Object x, Object y, Object z, Object A, Object i1, Object 
                          i2, Object i3, Object showFlag)
    {
      return mcr.EvaluateFunction("robot_arm_lin", x, y, z, A, i1, i2, i3, showFlag);
    }


    /// <summary>
    /// Provides the standard 0-input Object interface to the robot_arm_lin MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ПЗК+изобразить модель робота
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public Object[] robot_arm_lin(int numArgsOut)
    {
      return mcr.EvaluateFunction(numArgsOut, "robot_arm_lin", new Object[]{});
    }


    /// <summary>
    /// Provides the standard 1-input Object interface to the robot_arm_lin MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ПЗК+изобразить модель робота
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="x">Input argument #1</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public Object[] robot_arm_lin(int numArgsOut, Object x)
    {
      return mcr.EvaluateFunction(numArgsOut, "robot_arm_lin", x);
    }


    /// <summary>
    /// Provides the standard 2-input Object interface to the robot_arm_lin MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ПЗК+изобразить модель робота
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="x">Input argument #1</param>
    /// <param name="y">Input argument #2</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public Object[] robot_arm_lin(int numArgsOut, Object x, Object y)
    {
      return mcr.EvaluateFunction(numArgsOut, "robot_arm_lin", x, y);
    }


    /// <summary>
    /// Provides the standard 3-input Object interface to the robot_arm_lin MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ПЗК+изобразить модель робота
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="x">Input argument #1</param>
    /// <param name="y">Input argument #2</param>
    /// <param name="z">Input argument #3</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public Object[] robot_arm_lin(int numArgsOut, Object x, Object y, Object z)
    {
      return mcr.EvaluateFunction(numArgsOut, "robot_arm_lin", x, y, z);
    }


    /// <summary>
    /// Provides the standard 4-input Object interface to the robot_arm_lin MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ПЗК+изобразить модель робота
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="x">Input argument #1</param>
    /// <param name="y">Input argument #2</param>
    /// <param name="z">Input argument #3</param>
    /// <param name="A">Input argument #4</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public Object[] robot_arm_lin(int numArgsOut, Object x, Object y, Object z, Object A)
    {
      return mcr.EvaluateFunction(numArgsOut, "robot_arm_lin", x, y, z, A);
    }


    /// <summary>
    /// Provides the standard 5-input Object interface to the robot_arm_lin MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ПЗК+изобразить модель робота
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="x">Input argument #1</param>
    /// <param name="y">Input argument #2</param>
    /// <param name="z">Input argument #3</param>
    /// <param name="A">Input argument #4</param>
    /// <param name="i1">Input argument #5</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public Object[] robot_arm_lin(int numArgsOut, Object x, Object y, Object z, Object A, 
                            Object i1)
    {
      return mcr.EvaluateFunction(numArgsOut, "robot_arm_lin", x, y, z, A, i1);
    }


    /// <summary>
    /// Provides the standard 6-input Object interface to the robot_arm_lin MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ПЗК+изобразить модель робота
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="x">Input argument #1</param>
    /// <param name="y">Input argument #2</param>
    /// <param name="z">Input argument #3</param>
    /// <param name="A">Input argument #4</param>
    /// <param name="i1">Input argument #5</param>
    /// <param name="i2">Input argument #6</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public Object[] robot_arm_lin(int numArgsOut, Object x, Object y, Object z, Object A, 
                            Object i1, Object i2)
    {
      return mcr.EvaluateFunction(numArgsOut, "robot_arm_lin", x, y, z, A, i1, i2);
    }


    /// <summary>
    /// Provides the standard 7-input Object interface to the robot_arm_lin MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ПЗК+изобразить модель робота
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="x">Input argument #1</param>
    /// <param name="y">Input argument #2</param>
    /// <param name="z">Input argument #3</param>
    /// <param name="A">Input argument #4</param>
    /// <param name="i1">Input argument #5</param>
    /// <param name="i2">Input argument #6</param>
    /// <param name="i3">Input argument #7</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public Object[] robot_arm_lin(int numArgsOut, Object x, Object y, Object z, Object A, 
                            Object i1, Object i2, Object i3)
    {
      return mcr.EvaluateFunction(numArgsOut, "robot_arm_lin", x, y, z, A, i1, i2, i3);
    }


    /// <summary>
    /// Provides the standard 8-input Object interface to the robot_arm_lin MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ПЗК+изобразить модель робота
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <param name="x">Input argument #1</param>
    /// <param name="y">Input argument #2</param>
    /// <param name="z">Input argument #3</param>
    /// <param name="A">Input argument #4</param>
    /// <param name="i1">Input argument #5</param>
    /// <param name="i2">Input argument #6</param>
    /// <param name="i3">Input argument #7</param>
    /// <param name="showFlag">Input argument #8</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public Object[] robot_arm_lin(int numArgsOut, Object x, Object y, Object z, Object A, 
                            Object i1, Object i2, Object i3, Object showFlag)
    {
      return mcr.EvaluateFunction(numArgsOut, "robot_arm_lin", x, y, z, A, i1, i2, i3, showFlag);
    }


    /// <summary>
    /// Provides an interface for the robot_arm_lin function in which the input and
    /// output
    /// arguments are specified as an array of Objects.
    /// </summary>
    /// <remarks>
    /// This method will allocate and return by reference the output argument
    /// array.<newpara></newpara>
    /// M-Documentation:
    /// ПЗК+изобразить модель робота
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return</param>
    /// <param name= "argsOut">Array of Object output arguments</param>
    /// <param name= "argsIn">Array of Object input arguments</param>
    /// <param name= "varArgsIn">Array of Object representing variable input
    /// arguments</param>
    ///
    [MATLABSignature("robot_arm_lin", 8, 7, 0)]
    protected void robot_arm_lin(int numArgsOut, ref Object[] argsOut, Object[] argsIn, params Object[] varArgsIn)
    {
        mcr.EvaluateFunctionForTypeSafeCall("robot_arm_lin", numArgsOut, ref argsOut, argsIn, varArgsIn);
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
