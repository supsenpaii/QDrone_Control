function check_yaw_only_fa(buildExecutable)
% Verify and locally build the yaw-only FA model with QGC telemetry removed.
if nargin==0,buildExecutable=false;end
folder=fileparts(mfilename('fullpath'));project=fileparts(folder);
cd(project);addpath(folder,fullfile(project,'optical_flow'),fullfile(project,'qgc_bridge'));
model='Stabilizer_QDrone_QGC_OF_RYPI_FA';
assert(~bdIsLoaded(model),'Close the FA Stabilizer before checking the updated file.');
clear qdrone_ry_pi_step QDroneRYPIBuildable qdrone_yaw_only_mex
test_roll_yaw_controller;
load_system(fullfile(project,[model '.mdl']));
cleanup=onCleanup(@()close_system(model,0));
qgcBlocks=find_system(model,'LookUnderMasks','all','FollowLinks','off', ...
    'RegExp','on','Name','^QGC');
assert(isempty(qgcBlocks),'QGC telemetry blocks remain in the FA model.');
s=[model '/STABILIZER - QDRONE'];
assert(getSimulinkBlockHandle([s '/Yaw Integral Controller'])~=-1,'Yaw controller missing.');
cfg=str2num(get_param([s '/RYPI Configuration'],'Value')); %#ok<ST2NM>
assert(numel(cfg)==8 && all(cfg([1 3 5 7])==0),'Reserved roll parameters must be zero.');
yawConfig=cfg([2 4 6 8]);
assert(isequal(yawConfig(:),[0.01;0.015;0.003;1]),'Unexpected yaw parameters.');
set_param(model,'SimulationCommand','update');
fprintf('PASS: FA yaw-only diagram, configuration and QGC removal.\n');
if buildExecutable
    set_param(model,'DownloadToTarget','off');slbuild(model);
    fprintf('PASS: FA yaw-only local build.\n');
end
clear cleanup
end
