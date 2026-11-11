function recorder=open_capture_tool(ip,recordFrames)
% Open the sensor-only target model and start the host receiver.
% Target QUARC Build/Start remain explicit actions in Simulink.
if nargin<1,ip='192.168.2.52';end
if nargin<2,recordFrames=true;end
folder=fileparts(mfilename('fullpath'));project=fileparts(folder);
addpath(folder,fullfile(project,'optical_flow'));cd(project);
modelFile=fullfile(project,'QDrone_Capture.slx');
if ~exist(modelFile,'file'),build_qdrone_capture(true,ip);end
open_system(modelFile);
recorder=start_qdrone_capture(ip,recordFrames);
fprintf('Use QUARC Build and Start on QDrone_Capture after stopping other targets.\n');
end
