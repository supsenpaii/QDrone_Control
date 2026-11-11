@echo off
setlocal
call "%~dp0..\scripts\native_env.cmd"
if errorlevel 1 exit /b 1
cd /d "%~dp0"
if not exist native mkdir native
cl /nologo /W4 /O2 /MD /LD qdrone_ry_pi_core.c /Fo:native\yaw_only_core.obj /link /OUT:native\qdrone_yaw_only_core.dll /IMPLIB:native\yaw_only_core.lib /EXPORT:qdrone_ry_core_step /EXPORT:qdrone_ry_core_aw
if errorlevel 1 exit /b 1
cl /nologo /W4 /O2 /MD /LD /I"%MATLAB_ROOT%\extern\include" qdrone_ry_pi_mex.c qdrone_ry_pi_core.c /Fo:native\ /link /EXPORT:mexFunction /LIBPATH:"%MATLAB_ROOT%\extern\lib\win64\microsoft" libmex.lib libmx.lib /OUT:qdrone_yaw_only_mex.mexw64 /IMPLIB:native\yaw_only_mex.lib
exit /b %errorlevel%
