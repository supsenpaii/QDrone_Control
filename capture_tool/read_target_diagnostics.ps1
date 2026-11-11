Add-Type -TypeDefinition @"
using System;using System.Runtime.InteropServices;using System.Text;
public static class QDroneReadDiagnostics {
const string L=@"C:\Program Files\Quanser\QUARC\quarc_target.dll";
[DllImport("kernel32.dll",CharSet=CharSet.Unicode)]static extern bool SetDllDirectory(string p);
[DllImport(L,CallingConvention=CallingConvention.Cdecl,CharSet=CharSet.Ansi)]static extern int target_connect(string u,int s,int r,out IntPtr t);
[DllImport(L,CallingConvention=CallingConvention.Cdecl)]static extern int target_close(IntPtr t);
[DllImport(L,CallingConvention=CallingConvention.Cdecl,CharSet=CharSet.Ansi)]static extern int target_upload_file(IntPtr t,string path,ref Upload cb,long context);
[UnmanagedFunctionPointer(CallingConvention.Cdecl)]delegate int Put(IntPtr t,long ctx,int n,IntPtr p);
[StructLayout(LayoutKind.Sequential)]struct Upload{public IntPtr put;}
static StringBuilder text=new StringBuilder();static Put put=(t,c,n,p)=>{byte[] b=new byte[n];Marshal.Copy(p,b,0,n);text.Append(Encoding.UTF8.GetString(b));if(text.Length>200000)text.Remove(0,text.Length-200000);return n;};
public static void Run(){SetDllDirectory(@"C:\Program Files\Quanser\QUARC");IntPtr t;int r=target_connect("tcpip://192.168.2.52:17000",65536,65536,out t);Console.WriteLine("Connect="+r);if(r<0)return;Upload cb=new Upload{put=Marshal.GetFunctionPointerForDelegate(put)};
foreach(string path in new string[]{"/proc/cpuinfo","/var/log/messages","/var/log/kern.log"}){
text.Clear();r=target_upload_file(t,path,ref cb,0);Console.WriteLine("READ "+path+" result="+r);
foreach(string line in text.ToString().Split('\n')){
if(path.Contains("cpuinfo")){if(line.StartsWith("model name")||line.StartsWith("flags")||line.StartsWith("vendor_id"))Console.WriteLine(line);}
else if(line.Contains("QDrone_Capture")||line.Contains("invalid opcode")||line.Contains("segfault"))Console.WriteLine(line);
}
}target_close(t);}
}
"@
[QDroneReadDiagnostics]::Run()
