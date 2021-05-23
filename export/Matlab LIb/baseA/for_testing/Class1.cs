/*
* MATLAB Compiler: 7.0 (R2018b)
* Date: Tue May 18 22:56:47 2021
* Arguments:
* "-B""macro_default""-W""dotnet:baseA,Class1,4.0,private""-T""link:lib""-d""G:\GoogleDisc
* k\Main\C#\Matlab
* LIb\baseA\for_testing""-v""class{Class1:G:\GoogleDisck\_online\SIENCE\Simulation_ma
* tlab\export\baseA.m,G:\GoogleDisck\_online\SIENCE\Simulation_matlab\export\begin.m,
* G:\GoogleDisck\_online\SIENCE\Simulation_matlab\export\CalculateI.m,G:\GoogleDisck\
* _online\SIENCE\Simulation_matlab\export\forwardKinematics.m,G:\GoogleDisck\_on
* line\SIENCE\Simulation_matlab\export\inverceKinematic.m,G:\GoogleDisck\_online\SIEN
* CE\Simulation_matlab\export\inverceKinematicI.m,G:\GoogleDisck\_online\SIENCE\Simul
* ation_matlab\export\robot_arm_I.m,G:\GoogleDisck\_online\SIENCE\Simulation_matlab\e
* xport\robot_arm_lin.m}"
*/
using System;
using System.Reflection;
using System.IO;
using MathWorks.MATLAB.NET.Arrays;
using MathWorks.MATLAB.NET.Utility;

#if SHARED
[assembly: System.Reflection.AssemblyKeyFile(@"")]
#endif

namespace baseA
{

  /// <summary>
  /// The Class1 class provides a CLS compliant, MWArray interface to the MATLAB
  /// functions contained in the files:
  /// <newpara></newpara>
  /// G:\GoogleDisck\учёба_online\SIENCE\Simulation_matlab\export\baseA.m
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

          string ctfFileName = "baseA.ctf";

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
    /// Provides a single output, 0-input MWArrayinterface to the baseA MATLAB function.
    /// </summary>
    /// <remarks>
    /// </remarks>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray baseA()
    {
      return mcr.EvaluateFunction("baseA", new MWArray[]{});
    }


    /// <summary>
    /// Provides the standard 0-input MWArray interface to the baseA MATLAB function.
    /// </summary>
    /// <remarks>
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] baseA(int numArgsOut)
    {
      return mcr.EvaluateFunction(numArgsOut, "baseA", new MWArray[]{});
    }


    /// <summary>
    /// Provides an interface for the baseA function in which the input and output
    /// arguments are specified as an array of MWArrays.
    /// </summary>
    /// <remarks>
    /// This method will allocate and return by reference the output argument
    /// array.<newpara></newpara>
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return</param>
    /// <param name= "argsOut">Array of MWArray output arguments</param>
    /// <param name= "argsIn">Array of MWArray input arguments</param>
    ///
    public void baseA(int numArgsOut, ref MWArray[] argsOut, MWArray[] argsIn)
    {
      mcr.EvaluateFunction("baseA", numArgsOut, ref argsOut, argsIn);
    }


    /// <summary>
    /// Provides a void output, 0-input MWArrayinterface to the begin MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// axes('Xlim',[-900 900], 'Ylim',[-900 900], 'Zlim',[-500 900]);
    /// </remarks>
    ///
    public void begin()
    {
      mcr.EvaluateFunction(0, "begin", new MWArray[]{});
    }


    /// <summary>
    /// Provides the standard 0-input MWArray interface to the begin MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// axes('Xlim',[-900 900], 'Ylim',[-900 900], 'Zlim',[-500 900]);
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] begin(int numArgsOut)
    {
      return mcr.EvaluateFunction(numArgsOut, "begin", new MWArray[]{});
    }


    /// <summary>
    /// Provides a single output, 0-input MWArrayinterface to the CalculateI MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// indicator wrist calculation
    /// </remarks>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray CalculateI()
    {
      return mcr.EvaluateFunction("CalculateI", new MWArray[]{});
    }


    /// <summary>
    /// Provides a single output, 1-input MWArrayinterface to the CalculateI MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// indicator wrist calculation
    /// </remarks>
    /// <param name="A1">Input argument #1</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray CalculateI(MWArray A1)
    {
      return mcr.EvaluateFunction("CalculateI", A1);
    }


    /// <summary>
    /// Provides a single output, 2-input MWArrayinterface to the CalculateI MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// indicator wrist calculation
    /// </remarks>
    /// <param name="A1">Input argument #1</param>
    /// <param name="A2">Input argument #2</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray CalculateI(MWArray A1, MWArray A2)
    {
      return mcr.EvaluateFunction("CalculateI", A1, A2);
    }


    /// <summary>
    /// Provides a single output, 3-input MWArrayinterface to the CalculateI MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// indicator wrist calculation
    /// </remarks>
    /// <param name="A1">Input argument #1</param>
    /// <param name="A2">Input argument #2</param>
    /// <param name="A3">Input argument #3</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray CalculateI(MWArray A1, MWArray A2, MWArray A3)
    {
      return mcr.EvaluateFunction("CalculateI", A1, A2, A3);
    }


    /// <summary>
    /// Provides a single output, 4-input MWArrayinterface to the CalculateI MATLAB
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
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray CalculateI(MWArray A1, MWArray A2, MWArray A3, MWArray A4)
    {
      return mcr.EvaluateFunction("CalculateI", A1, A2, A3, A4);
    }


    /// <summary>
    /// Provides a single output, 5-input MWArrayinterface to the CalculateI MATLAB
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
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray CalculateI(MWArray A1, MWArray A2, MWArray A3, MWArray A4, MWArray A5)
    {
      return mcr.EvaluateFunction("CalculateI", A1, A2, A3, A4, A5);
    }


    /// <summary>
    /// Provides a single output, 6-input MWArrayinterface to the CalculateI MATLAB
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
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray CalculateI(MWArray A1, MWArray A2, MWArray A3, MWArray A4, MWArray A5, 
                        MWArray A6)
    {
      return mcr.EvaluateFunction("CalculateI", A1, A2, A3, A4, A5, A6);
    }


    /// <summary>
    /// Provides the standard 0-input MWArray interface to the CalculateI MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// indicator wrist calculation
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return.</param>
    /// <returns>An Array of length "numArgsOut" containing the output
    /// arguments.</returns>
    ///
    public MWArray[] CalculateI(int numArgsOut)
    {
      return mcr.EvaluateFunction(numArgsOut, "CalculateI", new MWArray[]{});
    }


    /// <summary>
    /// Provides the standard 1-input MWArray interface to the CalculateI MATLAB
    /// function.
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
    public MWArray[] CalculateI(int numArgsOut, MWArray A1)
    {
      return mcr.EvaluateFunction(numArgsOut, "CalculateI", A1);
    }


    /// <summary>
    /// Provides the standard 2-input MWArray interface to the CalculateI MATLAB
    /// function.
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
    public MWArray[] CalculateI(int numArgsOut, MWArray A1, MWArray A2)
    {
      return mcr.EvaluateFunction(numArgsOut, "CalculateI", A1, A2);
    }


    /// <summary>
    /// Provides the standard 3-input MWArray interface to the CalculateI MATLAB
    /// function.
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
    public MWArray[] CalculateI(int numArgsOut, MWArray A1, MWArray A2, MWArray A3)
    {
      return mcr.EvaluateFunction(numArgsOut, "CalculateI", A1, A2, A3);
    }


    /// <summary>
    /// Provides the standard 4-input MWArray interface to the CalculateI MATLAB
    /// function.
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
    public MWArray[] CalculateI(int numArgsOut, MWArray A1, MWArray A2, MWArray A3, 
                          MWArray A4)
    {
      return mcr.EvaluateFunction(numArgsOut, "CalculateI", A1, A2, A3, A4);
    }


    /// <summary>
    /// Provides the standard 5-input MWArray interface to the CalculateI MATLAB
    /// function.
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
    public MWArray[] CalculateI(int numArgsOut, MWArray A1, MWArray A2, MWArray A3, 
                          MWArray A4, MWArray A5)
    {
      return mcr.EvaluateFunction(numArgsOut, "CalculateI", A1, A2, A3, A4, A5);
    }


    /// <summary>
    /// Provides the standard 6-input MWArray interface to the CalculateI MATLAB
    /// function.
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
    public MWArray[] CalculateI(int numArgsOut, MWArray A1, MWArray A2, MWArray A3, 
                          MWArray A4, MWArray A5, MWArray A6)
    {
      return mcr.EvaluateFunction(numArgsOut, "CalculateI", A1, A2, A3, A4, A5, A6);
    }


    /// <summary>
    /// Provides an interface for the CalculateI function in which the input and output
    /// arguments are specified as an array of MWArrays.
    /// </summary>
    /// <remarks>
    /// This method will allocate and return by reference the output argument
    /// array.<newpara></newpara>
    /// M-Documentation:
    /// indicator wrist calculation
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return</param>
    /// <param name= "argsOut">Array of MWArray output arguments</param>
    /// <param name= "argsIn">Array of MWArray input arguments</param>
    ///
    public void CalculateI(int numArgsOut, ref MWArray[] argsOut, MWArray[] argsIn)
    {
      mcr.EvaluateFunction("CalculateI", numArgsOut, ref argsOut, argsIn);
    }


    /// <summary>
    /// Provides a single output, 0-input MWArrayinterface to the forwardKinematics
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ОЗК + изобразить модель робота при showFlag=1, reachFlag =0 в слечае
    /// невозможности достижения точки
    /// </remarks>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray forwardKinematics()
    {
      return mcr.EvaluateFunction("forwardKinematics", new MWArray[]{});
    }


    /// <summary>
    /// Provides a single output, 1-input MWArrayinterface to the forwardKinematics
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ОЗК + изобразить модель робота при showFlag=1, reachFlag =0 в слечае
    /// невозможности достижения точки
    /// </remarks>
    /// <param name="A1">Input argument #1</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray forwardKinematics(MWArray A1)
    {
      return mcr.EvaluateFunction("forwardKinematics", A1);
    }


    /// <summary>
    /// Provides a single output, 2-input MWArrayinterface to the forwardKinematics
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ОЗК + изобразить модель робота при showFlag=1, reachFlag =0 в слечае
    /// невозможности достижения точки
    /// </remarks>
    /// <param name="A1">Input argument #1</param>
    /// <param name="A2">Input argument #2</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray forwardKinematics(MWArray A1, MWArray A2)
    {
      return mcr.EvaluateFunction("forwardKinematics", A1, A2);
    }


    /// <summary>
    /// Provides a single output, 3-input MWArrayinterface to the forwardKinematics
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ОЗК + изобразить модель робота при showFlag=1, reachFlag =0 в слечае
    /// невозможности достижения точки
    /// </remarks>
    /// <param name="A1">Input argument #1</param>
    /// <param name="A2">Input argument #2</param>
    /// <param name="A3">Input argument #3</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray forwardKinematics(MWArray A1, MWArray A2, MWArray A3)
    {
      return mcr.EvaluateFunction("forwardKinematics", A1, A2, A3);
    }


    /// <summary>
    /// Provides a single output, 4-input MWArrayinterface to the forwardKinematics
    /// MATLAB function.
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
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray forwardKinematics(MWArray A1, MWArray A2, MWArray A3, MWArray A4)
    {
      return mcr.EvaluateFunction("forwardKinematics", A1, A2, A3, A4);
    }


    /// <summary>
    /// Provides a single output, 5-input MWArrayinterface to the forwardKinematics
    /// MATLAB function.
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
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray forwardKinematics(MWArray A1, MWArray A2, MWArray A3, MWArray A4, 
                               MWArray A5)
    {
      return mcr.EvaluateFunction("forwardKinematics", A1, A2, A3, A4, A5);
    }


    /// <summary>
    /// Provides a single output, 6-input MWArrayinterface to the forwardKinematics
    /// MATLAB function.
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
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray forwardKinematics(MWArray A1, MWArray A2, MWArray A3, MWArray A4, 
                               MWArray A5, MWArray A6)
    {
      return mcr.EvaluateFunction("forwardKinematics", A1, A2, A3, A4, A5, A6);
    }


    /// <summary>
    /// Provides the standard 0-input MWArray interface to the forwardKinematics MATLAB
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
    public MWArray[] forwardKinematics(int numArgsOut)
    {
      return mcr.EvaluateFunction(numArgsOut, "forwardKinematics", new MWArray[]{});
    }


    /// <summary>
    /// Provides the standard 1-input MWArray interface to the forwardKinematics MATLAB
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
    public MWArray[] forwardKinematics(int numArgsOut, MWArray A1)
    {
      return mcr.EvaluateFunction(numArgsOut, "forwardKinematics", A1);
    }


    /// <summary>
    /// Provides the standard 2-input MWArray interface to the forwardKinematics MATLAB
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
    public MWArray[] forwardKinematics(int numArgsOut, MWArray A1, MWArray A2)
    {
      return mcr.EvaluateFunction(numArgsOut, "forwardKinematics", A1, A2);
    }


    /// <summary>
    /// Provides the standard 3-input MWArray interface to the forwardKinematics MATLAB
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
    public MWArray[] forwardKinematics(int numArgsOut, MWArray A1, MWArray A2, MWArray A3)
    {
      return mcr.EvaluateFunction(numArgsOut, "forwardKinematics", A1, A2, A3);
    }


    /// <summary>
    /// Provides the standard 4-input MWArray interface to the forwardKinematics MATLAB
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
    public MWArray[] forwardKinematics(int numArgsOut, MWArray A1, MWArray A2, MWArray 
                                 A3, MWArray A4)
    {
      return mcr.EvaluateFunction(numArgsOut, "forwardKinematics", A1, A2, A3, A4);
    }


    /// <summary>
    /// Provides the standard 5-input MWArray interface to the forwardKinematics MATLAB
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
    public MWArray[] forwardKinematics(int numArgsOut, MWArray A1, MWArray A2, MWArray 
                                 A3, MWArray A4, MWArray A5)
    {
      return mcr.EvaluateFunction(numArgsOut, "forwardKinematics", A1, A2, A3, A4, A5);
    }


    /// <summary>
    /// Provides the standard 6-input MWArray interface to the forwardKinematics MATLAB
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
    public MWArray[] forwardKinematics(int numArgsOut, MWArray A1, MWArray A2, MWArray 
                                 A3, MWArray A4, MWArray A5, MWArray A6)
    {
      return mcr.EvaluateFunction(numArgsOut, "forwardKinematics", A1, A2, A3, A4, A5, A6);
    }


    /// <summary>
    /// Provides an interface for the forwardKinematics function in which the input and
    /// output
    /// arguments are specified as an array of MWArrays.
    /// </summary>
    /// <remarks>
    /// This method will allocate and return by reference the output argument
    /// array.<newpara></newpara>
    /// M-Documentation:
    /// ОЗК + изобразить модель робота при showFlag=1, reachFlag =0 в слечае
    /// невозможности достижения точки
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return</param>
    /// <param name= "argsOut">Array of MWArray output arguments</param>
    /// <param name= "argsIn">Array of MWArray input arguments</param>
    ///
    public void forwardKinematics(int numArgsOut, ref MWArray[] argsOut, MWArray[] argsIn)
    {
      mcr.EvaluateFunction("forwardKinematics", numArgsOut, ref argsOut, argsIn);
    }


    /// <summary>
    /// Provides a single output, 0-input MWArrayinterface to the inverceKinematic MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// d=[0;150;0;430;0;60];
    /// a=[0;430;-20;0;0;0];
    /// </remarks>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray inverceKinematic()
    {
      return mcr.EvaluateFunction("inverceKinematic", new MWArray[]{});
    }


    /// <summary>
    /// Provides a single output, 1-input MWArrayinterface to the inverceKinematic MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// d=[0;150;0;430;0;60];
    /// a=[0;430;-20;0;0;0];
    /// </remarks>
    /// <param name="x">Input argument #1</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray inverceKinematic(MWArray x)
    {
      return mcr.EvaluateFunction("inverceKinematic", x);
    }


    /// <summary>
    /// Provides a single output, 2-input MWArrayinterface to the inverceKinematic MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// d=[0;150;0;430;0;60];
    /// a=[0;430;-20;0;0;0];
    /// </remarks>
    /// <param name="x">Input argument #1</param>
    /// <param name="y">Input argument #2</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray inverceKinematic(MWArray x, MWArray y)
    {
      return mcr.EvaluateFunction("inverceKinematic", x, y);
    }


    /// <summary>
    /// Provides a single output, 3-input MWArrayinterface to the inverceKinematic MATLAB
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
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray inverceKinematic(MWArray x, MWArray y, MWArray z)
    {
      return mcr.EvaluateFunction("inverceKinematic", x, y, z);
    }


    /// <summary>
    /// Provides a single output, 4-input MWArrayinterface to the inverceKinematic MATLAB
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
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray inverceKinematic(MWArray x, MWArray y, MWArray z, MWArray A)
    {
      return mcr.EvaluateFunction("inverceKinematic", x, y, z, A);
    }


    /// <summary>
    /// Provides the standard 0-input MWArray interface to the inverceKinematic MATLAB
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
    public MWArray[] inverceKinematic(int numArgsOut)
    {
      return mcr.EvaluateFunction(numArgsOut, "inverceKinematic", new MWArray[]{});
    }


    /// <summary>
    /// Provides the standard 1-input MWArray interface to the inverceKinematic MATLAB
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
    public MWArray[] inverceKinematic(int numArgsOut, MWArray x)
    {
      return mcr.EvaluateFunction(numArgsOut, "inverceKinematic", x);
    }


    /// <summary>
    /// Provides the standard 2-input MWArray interface to the inverceKinematic MATLAB
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
    public MWArray[] inverceKinematic(int numArgsOut, MWArray x, MWArray y)
    {
      return mcr.EvaluateFunction(numArgsOut, "inverceKinematic", x, y);
    }


    /// <summary>
    /// Provides the standard 3-input MWArray interface to the inverceKinematic MATLAB
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
    public MWArray[] inverceKinematic(int numArgsOut, MWArray x, MWArray y, MWArray z)
    {
      return mcr.EvaluateFunction(numArgsOut, "inverceKinematic", x, y, z);
    }


    /// <summary>
    /// Provides the standard 4-input MWArray interface to the inverceKinematic MATLAB
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
    public MWArray[] inverceKinematic(int numArgsOut, MWArray x, MWArray y, MWArray z, 
                                MWArray A)
    {
      return mcr.EvaluateFunction(numArgsOut, "inverceKinematic", x, y, z, A);
    }


    /// <summary>
    /// Provides an interface for the inverceKinematic function in which the input and
    /// output
    /// arguments are specified as an array of MWArrays.
    /// </summary>
    /// <remarks>
    /// This method will allocate and return by reference the output argument
    /// array.<newpara></newpara>
    /// M-Documentation:
    /// d=[0;150;0;430;0;60];
    /// a=[0;430;-20;0;0;0];
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return</param>
    /// <param name= "argsOut">Array of MWArray output arguments</param>
    /// <param name= "argsIn">Array of MWArray input arguments</param>
    ///
    public void inverceKinematic(int numArgsOut, ref MWArray[] argsOut, MWArray[] argsIn)
    {
      mcr.EvaluateFunction("inverceKinematic", numArgsOut, ref argsOut, argsIn);
    }


    /// <summary>
    /// Provides a single output, 0-input MWArrayinterface to the inverceKinematicI
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// d=[0;150;0;430;0;60];
    /// a=[0;430;-20;0;0;0];
    /// </remarks>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray inverceKinematicI()
    {
      return mcr.EvaluateFunction("inverceKinematicI", new MWArray[]{});
    }


    /// <summary>
    /// Provides a single output, 1-input MWArrayinterface to the inverceKinematicI
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// d=[0;150;0;430;0;60];
    /// a=[0;430;-20;0;0;0];
    /// </remarks>
    /// <param name="x">Input argument #1</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray inverceKinematicI(MWArray x)
    {
      return mcr.EvaluateFunction("inverceKinematicI", x);
    }


    /// <summary>
    /// Provides a single output, 2-input MWArrayinterface to the inverceKinematicI
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// d=[0;150;0;430;0;60];
    /// a=[0;430;-20;0;0;0];
    /// </remarks>
    /// <param name="x">Input argument #1</param>
    /// <param name="y">Input argument #2</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray inverceKinematicI(MWArray x, MWArray y)
    {
      return mcr.EvaluateFunction("inverceKinematicI", x, y);
    }


    /// <summary>
    /// Provides a single output, 3-input MWArrayinterface to the inverceKinematicI
    /// MATLAB function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// d=[0;150;0;430;0;60];
    /// a=[0;430;-20;0;0;0];
    /// </remarks>
    /// <param name="x">Input argument #1</param>
    /// <param name="y">Input argument #2</param>
    /// <param name="z">Input argument #3</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray inverceKinematicI(MWArray x, MWArray y, MWArray z)
    {
      return mcr.EvaluateFunction("inverceKinematicI", x, y, z);
    }


    /// <summary>
    /// Provides a single output, 4-input MWArrayinterface to the inverceKinematicI
    /// MATLAB function.
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
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray inverceKinematicI(MWArray x, MWArray y, MWArray z, MWArray A)
    {
      return mcr.EvaluateFunction("inverceKinematicI", x, y, z, A);
    }


    /// <summary>
    /// Provides a single output, 5-input MWArrayinterface to the inverceKinematicI
    /// MATLAB function.
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
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray inverceKinematicI(MWArray x, MWArray y, MWArray z, MWArray A, MWArray 
                               sign_hand)
    {
      return mcr.EvaluateFunction("inverceKinematicI", x, y, z, A, sign_hand);
    }


    /// <summary>
    /// Provides a single output, 6-input MWArrayinterface to the inverceKinematicI
    /// MATLAB function.
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
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray inverceKinematicI(MWArray x, MWArray y, MWArray z, MWArray A, MWArray 
                               sign_hand, MWArray sign_elbow)
    {
      return mcr.EvaluateFunction("inverceKinematicI", x, y, z, A, sign_hand, sign_elbow);
    }


    /// <summary>
    /// Provides a single output, 7-input MWArrayinterface to the inverceKinematicI
    /// MATLAB function.
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
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray inverceKinematicI(MWArray x, MWArray y, MWArray z, MWArray A, MWArray 
                               sign_hand, MWArray sign_elbow, MWArray or_ind)
    {
      return mcr.EvaluateFunction("inverceKinematicI", x, y, z, A, sign_hand, sign_elbow, or_ind);
    }


    /// <summary>
    /// Provides the standard 0-input MWArray interface to the inverceKinematicI MATLAB
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
    public MWArray[] inverceKinematicI(int numArgsOut)
    {
      return mcr.EvaluateFunction(numArgsOut, "inverceKinematicI", new MWArray[]{});
    }


    /// <summary>
    /// Provides the standard 1-input MWArray interface to the inverceKinematicI MATLAB
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
    public MWArray[] inverceKinematicI(int numArgsOut, MWArray x)
    {
      return mcr.EvaluateFunction(numArgsOut, "inverceKinematicI", x);
    }


    /// <summary>
    /// Provides the standard 2-input MWArray interface to the inverceKinematicI MATLAB
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
    public MWArray[] inverceKinematicI(int numArgsOut, MWArray x, MWArray y)
    {
      return mcr.EvaluateFunction(numArgsOut, "inverceKinematicI", x, y);
    }


    /// <summary>
    /// Provides the standard 3-input MWArray interface to the inverceKinematicI MATLAB
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
    public MWArray[] inverceKinematicI(int numArgsOut, MWArray x, MWArray y, MWArray z)
    {
      return mcr.EvaluateFunction(numArgsOut, "inverceKinematicI", x, y, z);
    }


    /// <summary>
    /// Provides the standard 4-input MWArray interface to the inverceKinematicI MATLAB
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
    public MWArray[] inverceKinematicI(int numArgsOut, MWArray x, MWArray y, MWArray z, 
                                 MWArray A)
    {
      return mcr.EvaluateFunction(numArgsOut, "inverceKinematicI", x, y, z, A);
    }


    /// <summary>
    /// Provides the standard 5-input MWArray interface to the inverceKinematicI MATLAB
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
    public MWArray[] inverceKinematicI(int numArgsOut, MWArray x, MWArray y, MWArray z, 
                                 MWArray A, MWArray sign_hand)
    {
      return mcr.EvaluateFunction(numArgsOut, "inverceKinematicI", x, y, z, A, sign_hand);
    }


    /// <summary>
    /// Provides the standard 6-input MWArray interface to the inverceKinematicI MATLAB
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
    public MWArray[] inverceKinematicI(int numArgsOut, MWArray x, MWArray y, MWArray z, 
                                 MWArray A, MWArray sign_hand, MWArray sign_elbow)
    {
      return mcr.EvaluateFunction(numArgsOut, "inverceKinematicI", x, y, z, A, sign_hand, sign_elbow);
    }


    /// <summary>
    /// Provides the standard 7-input MWArray interface to the inverceKinematicI MATLAB
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
    public MWArray[] inverceKinematicI(int numArgsOut, MWArray x, MWArray y, MWArray z, 
                                 MWArray A, MWArray sign_hand, MWArray sign_elbow, 
                                 MWArray or_ind)
    {
      return mcr.EvaluateFunction(numArgsOut, "inverceKinematicI", x, y, z, A, sign_hand, sign_elbow, or_ind);
    }


    /// <summary>
    /// Provides an interface for the inverceKinematicI function in which the input and
    /// output
    /// arguments are specified as an array of MWArrays.
    /// </summary>
    /// <remarks>
    /// This method will allocate and return by reference the output argument
    /// array.<newpara></newpara>
    /// M-Documentation:
    /// d=[0;150;0;430;0;60];
    /// a=[0;430;-20;0;0;0];
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return</param>
    /// <param name= "argsOut">Array of MWArray output arguments</param>
    /// <param name= "argsIn">Array of MWArray input arguments</param>
    ///
    public void inverceKinematicI(int numArgsOut, ref MWArray[] argsOut, MWArray[] argsIn)
    {
      mcr.EvaluateFunction("inverceKinematicI", numArgsOut, ref argsOut, argsIn);
    }


    /// <summary>
    /// Provides a single output, 0-input MWArrayinterface to the robot_arm_I MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ПЗК+изобразить модель робота
    /// </remarks>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray robot_arm_I()
    {
      return mcr.EvaluateFunction("robot_arm_I", new MWArray[]{});
    }


    /// <summary>
    /// Provides a single output, 1-input MWArrayinterface to the robot_arm_I MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ПЗК+изобразить модель робота
    /// </remarks>
    /// <param name="A1">Input argument #1</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray robot_arm_I(MWArray A1)
    {
      return mcr.EvaluateFunction("robot_arm_I", A1);
    }


    /// <summary>
    /// Provides a single output, 2-input MWArrayinterface to the robot_arm_I MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ПЗК+изобразить модель робота
    /// </remarks>
    /// <param name="A1">Input argument #1</param>
    /// <param name="A2">Input argument #2</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray robot_arm_I(MWArray A1, MWArray A2)
    {
      return mcr.EvaluateFunction("robot_arm_I", A1, A2);
    }


    /// <summary>
    /// Provides a single output, 3-input MWArrayinterface to the robot_arm_I MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ПЗК+изобразить модель робота
    /// </remarks>
    /// <param name="A1">Input argument #1</param>
    /// <param name="A2">Input argument #2</param>
    /// <param name="A3">Input argument #3</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray robot_arm_I(MWArray A1, MWArray A2, MWArray A3)
    {
      return mcr.EvaluateFunction("robot_arm_I", A1, A2, A3);
    }


    /// <summary>
    /// Provides a single output, 4-input MWArrayinterface to the robot_arm_I MATLAB
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
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray robot_arm_I(MWArray A1, MWArray A2, MWArray A3, MWArray A4)
    {
      return mcr.EvaluateFunction("robot_arm_I", A1, A2, A3, A4);
    }


    /// <summary>
    /// Provides a single output, 5-input MWArrayinterface to the robot_arm_I MATLAB
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
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray robot_arm_I(MWArray A1, MWArray A2, MWArray A3, MWArray A4, MWArray A5)
    {
      return mcr.EvaluateFunction("robot_arm_I", A1, A2, A3, A4, A5);
    }


    /// <summary>
    /// Provides a single output, 6-input MWArrayinterface to the robot_arm_I MATLAB
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
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray robot_arm_I(MWArray A1, MWArray A2, MWArray A3, MWArray A4, MWArray 
                         A5, MWArray A6)
    {
      return mcr.EvaluateFunction("robot_arm_I", A1, A2, A3, A4, A5, A6);
    }


    /// <summary>
    /// Provides a single output, 7-input MWArrayinterface to the robot_arm_I MATLAB
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
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray robot_arm_I(MWArray A1, MWArray A2, MWArray A3, MWArray A4, MWArray 
                         A5, MWArray A6, MWArray showFlag)
    {
      return mcr.EvaluateFunction("robot_arm_I", A1, A2, A3, A4, A5, A6, showFlag);
    }


    /// <summary>
    /// Provides the standard 0-input MWArray interface to the robot_arm_I MATLAB
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
    public MWArray[] robot_arm_I(int numArgsOut)
    {
      return mcr.EvaluateFunction(numArgsOut, "robot_arm_I", new MWArray[]{});
    }


    /// <summary>
    /// Provides the standard 1-input MWArray interface to the robot_arm_I MATLAB
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
    public MWArray[] robot_arm_I(int numArgsOut, MWArray A1)
    {
      return mcr.EvaluateFunction(numArgsOut, "robot_arm_I", A1);
    }


    /// <summary>
    /// Provides the standard 2-input MWArray interface to the robot_arm_I MATLAB
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
    public MWArray[] robot_arm_I(int numArgsOut, MWArray A1, MWArray A2)
    {
      return mcr.EvaluateFunction(numArgsOut, "robot_arm_I", A1, A2);
    }


    /// <summary>
    /// Provides the standard 3-input MWArray interface to the robot_arm_I MATLAB
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
    public MWArray[] robot_arm_I(int numArgsOut, MWArray A1, MWArray A2, MWArray A3)
    {
      return mcr.EvaluateFunction(numArgsOut, "robot_arm_I", A1, A2, A3);
    }


    /// <summary>
    /// Provides the standard 4-input MWArray interface to the robot_arm_I MATLAB
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
    public MWArray[] robot_arm_I(int numArgsOut, MWArray A1, MWArray A2, MWArray A3, 
                           MWArray A4)
    {
      return mcr.EvaluateFunction(numArgsOut, "robot_arm_I", A1, A2, A3, A4);
    }


    /// <summary>
    /// Provides the standard 5-input MWArray interface to the robot_arm_I MATLAB
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
    public MWArray[] robot_arm_I(int numArgsOut, MWArray A1, MWArray A2, MWArray A3, 
                           MWArray A4, MWArray A5)
    {
      return mcr.EvaluateFunction(numArgsOut, "robot_arm_I", A1, A2, A3, A4, A5);
    }


    /// <summary>
    /// Provides the standard 6-input MWArray interface to the robot_arm_I MATLAB
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
    public MWArray[] robot_arm_I(int numArgsOut, MWArray A1, MWArray A2, MWArray A3, 
                           MWArray A4, MWArray A5, MWArray A6)
    {
      return mcr.EvaluateFunction(numArgsOut, "robot_arm_I", A1, A2, A3, A4, A5, A6);
    }


    /// <summary>
    /// Provides the standard 7-input MWArray interface to the robot_arm_I MATLAB
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
    public MWArray[] robot_arm_I(int numArgsOut, MWArray A1, MWArray A2, MWArray A3, 
                           MWArray A4, MWArray A5, MWArray A6, MWArray showFlag)
    {
      return mcr.EvaluateFunction(numArgsOut, "robot_arm_I", A1, A2, A3, A4, A5, A6, showFlag);
    }


    /// <summary>
    /// Provides an interface for the robot_arm_I function in which the input and output
    /// arguments are specified as an array of MWArrays.
    /// </summary>
    /// <remarks>
    /// This method will allocate and return by reference the output argument
    /// array.<newpara></newpara>
    /// M-Documentation:
    /// ПЗК+изобразить модель робота
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return</param>
    /// <param name= "argsOut">Array of MWArray output arguments</param>
    /// <param name= "argsIn">Array of MWArray input arguments</param>
    ///
    public void robot_arm_I(int numArgsOut, ref MWArray[] argsOut, MWArray[] argsIn)
    {
      mcr.EvaluateFunction("robot_arm_I", numArgsOut, ref argsOut, argsIn);
    }


    /// <summary>
    /// Provides a single output, 0-input MWArrayinterface to the robot_arm_lin MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ПЗК+изобразить модель робота
    /// </remarks>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray robot_arm_lin()
    {
      return mcr.EvaluateFunction("robot_arm_lin", new MWArray[]{});
    }


    /// <summary>
    /// Provides a single output, 1-input MWArrayinterface to the robot_arm_lin MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ПЗК+изобразить модель робота
    /// </remarks>
    /// <param name="x">Input argument #1</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray robot_arm_lin(MWArray x)
    {
      return mcr.EvaluateFunction("robot_arm_lin", x);
    }


    /// <summary>
    /// Provides a single output, 2-input MWArrayinterface to the robot_arm_lin MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ПЗК+изобразить модель робота
    /// </remarks>
    /// <param name="x">Input argument #1</param>
    /// <param name="y">Input argument #2</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray robot_arm_lin(MWArray x, MWArray y)
    {
      return mcr.EvaluateFunction("robot_arm_lin", x, y);
    }


    /// <summary>
    /// Provides a single output, 3-input MWArrayinterface to the robot_arm_lin MATLAB
    /// function.
    /// </summary>
    /// <remarks>
    /// M-Documentation:
    /// ПЗК+изобразить модель робота
    /// </remarks>
    /// <param name="x">Input argument #1</param>
    /// <param name="y">Input argument #2</param>
    /// <param name="z">Input argument #3</param>
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray robot_arm_lin(MWArray x, MWArray y, MWArray z)
    {
      return mcr.EvaluateFunction("robot_arm_lin", x, y, z);
    }


    /// <summary>
    /// Provides a single output, 4-input MWArrayinterface to the robot_arm_lin MATLAB
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
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray robot_arm_lin(MWArray x, MWArray y, MWArray z, MWArray A)
    {
      return mcr.EvaluateFunction("robot_arm_lin", x, y, z, A);
    }


    /// <summary>
    /// Provides a single output, 5-input MWArrayinterface to the robot_arm_lin MATLAB
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
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray robot_arm_lin(MWArray x, MWArray y, MWArray z, MWArray A, MWArray i1)
    {
      return mcr.EvaluateFunction("robot_arm_lin", x, y, z, A, i1);
    }


    /// <summary>
    /// Provides a single output, 6-input MWArrayinterface to the robot_arm_lin MATLAB
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
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray robot_arm_lin(MWArray x, MWArray y, MWArray z, MWArray A, MWArray i1, 
                           MWArray i2)
    {
      return mcr.EvaluateFunction("robot_arm_lin", x, y, z, A, i1, i2);
    }


    /// <summary>
    /// Provides a single output, 7-input MWArrayinterface to the robot_arm_lin MATLAB
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
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray robot_arm_lin(MWArray x, MWArray y, MWArray z, MWArray A, MWArray i1, 
                           MWArray i2, MWArray i3)
    {
      return mcr.EvaluateFunction("robot_arm_lin", x, y, z, A, i1, i2, i3);
    }


    /// <summary>
    /// Provides a single output, 8-input MWArrayinterface to the robot_arm_lin MATLAB
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
    /// <returns>An MWArray containing the first output argument.</returns>
    ///
    public MWArray robot_arm_lin(MWArray x, MWArray y, MWArray z, MWArray A, MWArray i1, 
                           MWArray i2, MWArray i3, MWArray showFlag)
    {
      return mcr.EvaluateFunction("robot_arm_lin", x, y, z, A, i1, i2, i3, showFlag);
    }


    /// <summary>
    /// Provides the standard 0-input MWArray interface to the robot_arm_lin MATLAB
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
    public MWArray[] robot_arm_lin(int numArgsOut)
    {
      return mcr.EvaluateFunction(numArgsOut, "robot_arm_lin", new MWArray[]{});
    }


    /// <summary>
    /// Provides the standard 1-input MWArray interface to the robot_arm_lin MATLAB
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
    public MWArray[] robot_arm_lin(int numArgsOut, MWArray x)
    {
      return mcr.EvaluateFunction(numArgsOut, "robot_arm_lin", x);
    }


    /// <summary>
    /// Provides the standard 2-input MWArray interface to the robot_arm_lin MATLAB
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
    public MWArray[] robot_arm_lin(int numArgsOut, MWArray x, MWArray y)
    {
      return mcr.EvaluateFunction(numArgsOut, "robot_arm_lin", x, y);
    }


    /// <summary>
    /// Provides the standard 3-input MWArray interface to the robot_arm_lin MATLAB
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
    public MWArray[] robot_arm_lin(int numArgsOut, MWArray x, MWArray y, MWArray z)
    {
      return mcr.EvaluateFunction(numArgsOut, "robot_arm_lin", x, y, z);
    }


    /// <summary>
    /// Provides the standard 4-input MWArray interface to the robot_arm_lin MATLAB
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
    public MWArray[] robot_arm_lin(int numArgsOut, MWArray x, MWArray y, MWArray z, 
                             MWArray A)
    {
      return mcr.EvaluateFunction(numArgsOut, "robot_arm_lin", x, y, z, A);
    }


    /// <summary>
    /// Provides the standard 5-input MWArray interface to the robot_arm_lin MATLAB
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
    public MWArray[] robot_arm_lin(int numArgsOut, MWArray x, MWArray y, MWArray z, 
                             MWArray A, MWArray i1)
    {
      return mcr.EvaluateFunction(numArgsOut, "robot_arm_lin", x, y, z, A, i1);
    }


    /// <summary>
    /// Provides the standard 6-input MWArray interface to the robot_arm_lin MATLAB
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
    public MWArray[] robot_arm_lin(int numArgsOut, MWArray x, MWArray y, MWArray z, 
                             MWArray A, MWArray i1, MWArray i2)
    {
      return mcr.EvaluateFunction(numArgsOut, "robot_arm_lin", x, y, z, A, i1, i2);
    }


    /// <summary>
    /// Provides the standard 7-input MWArray interface to the robot_arm_lin MATLAB
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
    public MWArray[] robot_arm_lin(int numArgsOut, MWArray x, MWArray y, MWArray z, 
                             MWArray A, MWArray i1, MWArray i2, MWArray i3)
    {
      return mcr.EvaluateFunction(numArgsOut, "robot_arm_lin", x, y, z, A, i1, i2, i3);
    }


    /// <summary>
    /// Provides the standard 8-input MWArray interface to the robot_arm_lin MATLAB
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
    public MWArray[] robot_arm_lin(int numArgsOut, MWArray x, MWArray y, MWArray z, 
                             MWArray A, MWArray i1, MWArray i2, MWArray i3, MWArray 
                             showFlag)
    {
      return mcr.EvaluateFunction(numArgsOut, "robot_arm_lin", x, y, z, A, i1, i2, i3, showFlag);
    }


    /// <summary>
    /// Provides an interface for the robot_arm_lin function in which the input and
    /// output
    /// arguments are specified as an array of MWArrays.
    /// </summary>
    /// <remarks>
    /// This method will allocate and return by reference the output argument
    /// array.<newpara></newpara>
    /// M-Documentation:
    /// ПЗК+изобразить модель робота
    /// </remarks>
    /// <param name="numArgsOut">The number of output arguments to return</param>
    /// <param name= "argsOut">Array of MWArray output arguments</param>
    /// <param name= "argsIn">Array of MWArray input arguments</param>
    ///
    public void robot_arm_lin(int numArgsOut, ref MWArray[] argsOut, MWArray[] argsIn)
    {
      mcr.EvaluateFunction("robot_arm_lin", numArgsOut, ref argsOut, argsIn);
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
