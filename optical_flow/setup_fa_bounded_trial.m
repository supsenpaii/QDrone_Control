function setup_fa_bounded_trial(buildExecutable)
% Prepare stage 4 with low authority after matching measured ground evidence.
% This function can Build locally. Download, Start and Arm are separate actions.
if nargin<1,buildExecutable=false;end
folder=fileparts(mfilename('fullpath'));project=fileparts(folder);cd(project);addpath(folder);
m='Stabilizer_QDrone_QGC_OF_RYPI_FA';load_system(fullfile(project,[m '.mdl']));
assert(strcmp(get_param(m,'SimulationStatus'),'stopped'),'Stop Stabilizer first.');
assert(strcmp(get_param(m,'Dirty'),'off'),'Save pending Stabilizer changes first.');
block=[m '/STABILIZER - QDRONE/FA Configuration'];
cfg=str2num(get_param(block,'Value')); %#ok<ST2NM>
assert(numel(cfg)==13&&cfg(1)==3,'Start from the tested stage 3 model.');
rotation=str2num(get_param([m '/STABILIZER - QDRONE/FA Camera To Body'],'Value')); %#ok<ST2NM>
evidence=load(fullfile(folder,'fa_timing_ground_verification.mat'),'verification');v=evidence.verification;
assert(v.passed&&v.ground_only&&v.image_delay_s==.034 ...
    &&isequal(v.config(:),cfg(:))&&isequal(v.cameraToBody,rotation), ...
    'Ground evidence must match the current shadow model.');
clear qdrone_fa_step qdrone_fa_timed_mex
copyfile(fullfile(folder,'native','qdrone_fa_timed_mex.mexw64'), ...
    fullfile(folder,'qdrone_fa_timed_mex.mexw64'),'f');rehash;
trial=cfg(:);trial(1)=4;trial(5)=.01;
trial(6)=.5*pi/180;trial(7)=pi/180;trial(13)=min(trial(13),8*pi/180);
% Exercise the new MEX and its Disarm gate before saving any active profile.
frameTime=0;command=zeros(3,1);flow=[0;0;1;1;.033;0;35;35];
for k=1:1200
    time=k*.001;if mod(k,33)==0,frameTime=time;end
    [out,d]=qdrone_fa_step(command,flow,frameTime,zeros(3,1),zeros(3,1), ...
        1,10,0,1,trial,rotation,time);
end
assert(d(1)==5&&isequal(out,command),'Bounded trial MEX check failed.');
[out,d]=qdrone_fa_step(command,flow,frameTime,zeros(3,1),zeros(3,1), ...
    0,0,0,1,trial,rotation,1.201);
assert(d(1)==2&&isequal(out,command),'Bounded trial Disarm gate failed.');
clear qdrone_fa_step
backup=fullfile(project,'_archive','fa_bounded_trial');
if ~isfolder(backup),mkdir(backup);end
copyfile(get_param(m,'FileName'),fullfile(backup,[m '_' datestr(now,'yyyymmdd_HHMMSS') '.mdl']));
set_param(block,'Value',mat2str(trial,17));set_param(m,'DownloadToTarget','off');
set_param(m,'SimulationCommand','update');save_system(m);
fprintf('PASS: stage 4 bounded active profile, cap .5 deg, slew 1 deg/s, tilt gate 8 deg.\n');
if buildExecutable
    slbuild(m);
    fprintf('PASS: bounded trial local build. Download, Start and Arm remain explicit.\n');
end
end
