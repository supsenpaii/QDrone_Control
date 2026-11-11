@echo off
setlocal
call "%~dp0..\scripts\native_env.cmd"
if errorlevel 1 exit /b 1
cd /d "%~dp0"
if not exist native mkdir native
cl /nologo /W4 /O2 /MD /LD qdrone_sb_mode_core.c /Fo:native\sb.obj /link /OUT:native\qdrone_sb_mode_core.dll /IMPLIB:native\sb.lib /EXPORT:qdrone_sb_mode
if errorlevel 1 exit /b 1
cl /nologo /W4 /O2 /MD /LD /I"%MATLAB_ROOT%\extern\include" qdrone_sb_mode_mex.c qdrone_sb_mode_core.c /Fo:native\ /link /EXPORT:mexFunction /LIBPATH:"%MATLAB_ROOT%\extern\lib\win64\microsoft" libmex.lib libmx.lib /OUT:qdrone_sb_mode_mex.mexw64 /IMPLIB:native\sb_mex.lib
exit /b %errorlevel%
