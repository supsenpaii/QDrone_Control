function setup_fa_timing_trial(buildExecutable)
% Stage 3 selects the timestamped-gyro, disarmed-only shadow trial.
if nargin<1,buildExecutable=false;end
folder=fileparts(mfilename('fullpath'));project=fileparts(folder);cd(project);addpath(folder);
m='Stabilizer_QDrone_QGC_OF_RYPI_FA';
load_system(fullfile(project,[m '.mdl']));
assert(strcmp(get_param(m,'SimulationStatus'),'stopped'),'Stop Stabilizer first.');
assert(strcmp(get_param(m,'Dirty'),'off'),'Save pending Stabilizer changes first.');
block=[m '/STABILIZER - QDRONE/FA Configuration'];
cfg=str2num(get_param(block,'Value')); %#ok<ST2NM>
assert(numel(cfg)==13&&ismember(cfg(1),[2 3]),'Start with a configured shadow model.');
assert(isfile(fullfile(folder,'qdrone_fa_timed_mex.mexw64')),'Build optical_flow/build_fa_native.cmd first.');
clear qdrone_fa_step qdrone_fa_timed_mex
staged=fullfile(folder,'native','qdrone_fa_timed_mex.mexw64');
if isfile(staged),copyfile(staged,fullfile(folder,'qdrone_fa_timed_mex.mexw64'),'f');rehash;end
test_flow_assist;
clear qdrone_fa_step
trial=cfg(:);trial(1)=3;
rotation=str2num(get_param([m '/STABILIZER - QDRONE/FA Camera To Body'],'Value')); %#ok<ST2NM>
frameTime=0;command=zeros(3,1);flow=[0;0;1;1;.033;0;35;35];
for k=1:1200
    time=k*.001;if mod(k,33)==0,frameTime=time;end
    [out,diagnostic]=qdrone_fa_step(command,flow,frameTime,zeros(3,1), ...
        zeros(3,1),0,0,0,1,trial,rotation,time);
end
assert(diagnostic(1)==9&&isequal(out,command),'Timed shadow MEX self-check failed.');
[out,diagnostic]=qdrone_fa_step(command,flow,frameTime,zeros(3,1), ...
    zeros(3,1),1,0,0,1,trial,rotation,1.201);
assert(diagnostic(1)==2&&isequal(out,command),'Timed shadow Arm gate failed.');
clear qdrone_fa_step
backup=fullfile(project,'_archive','fa_timing_trial');
if ~isfolder(backup),mkdir(backup);end
copyfile(get_param(m,'FileName'),fullfile(backup,[m '_' datestr(now,'yyyymmdd_HHMMSS') '.mdl']));
cfg(1)=3;
set_param(block,'Value',mat2str(cfg,17));
set_param(m,'DownloadToTarget','off');
set_param(m,'SimulationCommand','update');save_system(m);
fprintf('PASS: stage 3 timestamped-gyro shadow trial, image delay 0.034 s.\n');
if buildExecutable
    slbuild(m);
    fprintf('PASS: timing trial local build. Download and Start remain explicit.\n');
end
end
