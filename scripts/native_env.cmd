@echo off
rem Caller uses setlocal. Configure MATLAB_ROOT and VCVARS64 to override defaults.
if not defined MATLAB_ROOT set "MATLAB_ROOT=C:\Program Files\MATLAB\R2019a"
if not exist "%MATLAB_ROOT%\extern\include\mex.h" (
  echo Set MATLAB_ROOT to your MATLAB installation directory.
  exit /b 1
)
if not defined VCVARS64 set "VCVARS64=C:\Program Files (x86)\Microsoft Visual Studio\2017\Community\VC\Auxiliary\Build\vcvars64.bat"
if not exist "%VCVARS64%" (
  echo Set VCVARS64 to the vcvars64.bat of a compatible Visual Studio installation.
  exit /b 1
)
call "%VCVARS64%"
exit /b %errorlevel%
