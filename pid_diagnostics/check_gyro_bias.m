function check_gyro_bias(buildExecutable)
% Check gyro compensation copy, optionally build locally; never Start/Arm.
if nargin==0, buildExecutable=false; end
check_pid_diagnostics(buildExecutable, 'Stabilizer_QDrone_048378_GyroCal');
end
