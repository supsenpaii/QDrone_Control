function setup_capture_calibration_inputs
% Patch the saved Capture model to use the proven six IMU input channels.
folder=fileparts(mfilename('fullpath'));project=fileparts(folder);
addpath(folder,fullfile(project,'optical_flow'));cd(project);
model='QDrone_Capture';load_system(fullfile(project,[model '.slx']));
assert(strcmp(get_param(model,'SimulationStatus'),'stopped'),'Stop QDrone_Capture first.');
assert(strcmp(get_param(model,'Dirty'),'off'),'Save or discard pending model changes first.');
file=get_param(model,'FileName');
copyfile(file,[file '.' datestr(now,'yyyymmdd_HHMMSS') '.imu6.bak']);
set_param([model '/All Sensor Inputs'],'other_channels','[3000:3002 4000:4002]');
set_param(model,'SimulationCommand','update');
save_system(model);
fprintf('PASS: six IMU channels, fixed 30-value packet, diagram update. Build/Download Capture next.\n');
end
