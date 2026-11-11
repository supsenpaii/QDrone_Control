$ErrorActionPreference='Stop'
Add-Type -TypeDefinition @"
using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
public static class QDroneHelperConsole {
 const string Lib=@"C:\Program Files\Quanser\QUARC\quarc_target.dll";
 [DllImport("kernel32.dll",CharSet=CharSet.Unicode)] static extern bool SetDllDirectory(string path);
 [DllImport(Lib,CallingConvention=CallingConvention.Cdecl,CharSet=CharSet.Ansi)] static extern int target_connect(string uri,int snd,int rcv,out IntPtr target);
 [DllImport(Lib,CallingConvention=CallingConvention.Cdecl)] static extern int target_close(IntPtr target);
 [DllImport(Lib,CallingConvention=CallingConvention.Cdecl,CharSet=CharSet.Ansi)] static extern int target_load_model(IntPtr target,string name,string arguments);
 [DllImport(Lib,CallingConvention=CallingConvention.Cdecl,CharSet=CharSet.Ansi)] static extern int target_is_model_loaded(IntPtr target,string name);
 [DllImport(Lib,CallingConvention=CallingConvention.Cdecl)] static extern int target_get_loaded_models(IntPtr target,ref Enumeration cb,long context);
 [DllImport(Lib,CallingConvention=CallingConvention.Cdecl,CharSet=CharSet.Ansi)] static extern int target_open_model_output(IntPtr target,string name,out long handle);
 [DllImport(Lib,CallingConvention=CallingConvention.Cdecl)] static extern int target_get_model_output(IntPtr target,long handle,ref Output cb,long context);
 [UnmanagedFunctionPointer(CallingConvention.Cdecl)] delegate int Enumerate(IntPtr target,long context,IntPtr model,IntPtr arguments);
 [UnmanagedFunctionPointer(CallingConvention.Cdecl)] delegate int ModelEvent(IntPtr target,long handle,long context,IntPtr model);
 [UnmanagedFunctionPointer(CallingConvention.Cdecl)] delegate int TextEvent(IntPtr target,long handle,long context,IntPtr model,int size,IntPtr text);
 [UnmanagedFunctionPointer(CallingConvention.Cdecl)] delegate int ExitEvent(IntPtr target,long handle,long context,IntPtr model,int code);
 [UnmanagedFunctionPointer(CallingConvention.Cdecl)] delegate int InitEvent(IntPtr target,long handle,long context);
 [StructLayout(LayoutKind.Sequential)] struct Enumeration {public IntPtr enumerate;}
 [StructLayout(LayoutKind.Sequential)] struct Output {public IntPtr downloaded,loading,text,terminated,initialize,loaded;}
 static List<string> models=new List<string>();
 static Enumerate enumerate=(t,c,m,a)=>{string name=Marshal.PtrToStringAnsi(m);models.Add(name);Console.WriteLine("Existing: "+name+" "+Marshal.PtrToStringAnsi(a));return 0;};
 static ModelEvent modelEvent=(t,h,c,m)=>{Console.WriteLine("Runtime event: "+Marshal.PtrToStringAnsi(m));return 0;};
 static TextEvent textEvent=(t,h,c,m,n,p)=>{byte[] bytes=new byte[n];Marshal.Copy(p,bytes,0,n);Console.Write(Encoding.UTF8.GetString(bytes));return 0;};
 static ExitEvent exitEvent=(t,h,c,m,e)=>{Console.WriteLine("Runtime terminated: "+Marshal.PtrToStringAnsi(m)+" exit="+e);return 0;};
 static InitEvent initEvent=(t,h,c)=>0;
 public static void Run(){
  SetDllDirectory(@"C:\Program Files\Quanser\QUARC");
  string uri="tcpip://192.168.2.52:17000";string name="QDrone_RuntimeProbe.rt-linux_x64";
  IntPtr control;int result=target_connect(uri,65536,65536,out control);Console.WriteLine("Manager connect="+result);if(result<0)return;
  Enumeration en=new Enumeration{enumerate=Marshal.GetFunctionPointerForDelegate(enumerate)};
  target_get_loaded_models(control,ref en,0);
  foreach(string model in models)if(!model.StartsWith("QDrone_Capture",StringComparison.OrdinalIgnoreCase)){
   Console.WriteLine("Another target is active. No model loaded or stopped.");target_close(control);return;
  }
  IntPtr output;result=target_connect(uri,65536,65536,out output);Console.WriteLine("Console connect="+result);if(result<0){target_close(control);return;}
  long handle;result=target_open_model_output(output,name,out handle);Console.WriteLine("Console open="+result);if(result<0){target_close(control);return;}
  Output callbacks=new Output{downloaded=Marshal.GetFunctionPointerForDelegate(modelEvent),loading=Marshal.GetFunctionPointerForDelegate(modelEvent),text=Marshal.GetFunctionPointerForDelegate(textEvent),terminated=Marshal.GetFunctionPointerForDelegate(exitEvent),initialize=Marshal.GetFunctionPointerForDelegate(initEvent),loaded=Marshal.GetFunctionPointerForDelegate(modelEvent)};
  Thread reader=new Thread(()=>{try{while(true){int code=target_get_model_output(output,handle,ref callbacks,0);if(code<0){Console.WriteLine("Console read="+code);break;}}}catch(Exception e){Console.WriteLine("Console error: "+e.Message);}});reader.IsBackground=true;reader.Start();
  Thread.Sleep(300);
  if(target_is_model_loaded(control,name)<=0){result=target_load_model(control,name,"-d /tmp");Console.WriteLine("Load sensor-only model="+result);}
  else Console.WriteLine("Capture already loaded; not loaded twice.");
  Thread.Sleep(6000);
  Console.WriteLine("Loaded after 6 seconds="+target_is_model_loaded(control,name));
  target_close(control);
  Console.WriteLine("Probe finished. No model stopped and no motor command sent.");
 }
}
"@
[QDroneHelperConsole]::Run()

