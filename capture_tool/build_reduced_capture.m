cd(fileparts(fileparts(mfilename('fullpath'))));addpath('capture_tool','optical_flow');
models=qc_get_loaded_models('tcpip://192.168.2.52:17000');
if ~isempty(models),disp(models);error('Other target is loaded; no build/download performed.');end
build_qdrone_capture(false);
fprintf('Building the sensor-only down-camera variant; no motor outputs.\n');
qc_build_model('QDrone_Capture');
fprintf('PASS: reduced capture variant built and downloaded. No start command sent.\n');
