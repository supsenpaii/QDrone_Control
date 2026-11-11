@echo off
setlocal
call "%~dp0..\scripts\native_env.cmd"
if errorlevel 1 exit /b 1
cd /d "%~dp0"
if not exist native mkdir native
cl /nologo /W4 /O2 /MD /LD qdrone_fa_core.c /Fo:native\fa.obj /link /OUT:native\qdrone_fa_core.dll /IMPLIB:native\fa.lib /EXPORT:qdrone_fa_step /EXPORT:qdrone_fa_step_timed
if errorlevel 1 exit /b 1
rem Stage the MEX so a running MATLAB can clear its previous version first.
cl /nologo /W4 /O2 /MD /LD /I"%MATLAB_ROOT%\extern\include" qdrone_fa_mex.c qdrone_fa_core.c /Fo:native\ /link /EXPORT:mexFunction /LIBPATH:"%MATLAB_ROOT%\extern\lib\win64\microsoft" libmex.lib libmx.lib /OUT:native\qdrone_fa_timed_mex.mexw64 /IMPLIB:native\fa_timed_mex.lib
if errorlevel 1 exit /b 1
copy /Y native\qdrone_fa_timed_mex.mexw64 qdrone_fa_timed_mex.mexw64 >nul 2>nul
if errorlevel 1 echo MEX is in use. The setup function will clear and install the staged MEX.
copy /Y native\qdrone_fa_timed_mex.mexw64 qdrone_fa_mex.mexw64 >nul
if errorlevel 1 exit /b 1
exit /b 0
