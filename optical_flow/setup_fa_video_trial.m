function setup_fa_video_trial(videoEnabled,buildExecutable)
% Compare task timing with/without the compressed preview sink.
% Camera capture, flow computation and numeric logs continue to execute.
if nargin<1,videoEnabled=false;end
if nargin<2,buildExecutable=true;end
validateattributes(videoEnabled,{'logical','numeric'},{'scalar','binary'});
folder=fileparts(mfilename('fullpath'));project=fileparts(folder);cd(project);addpath(folder);
m='Stabilizer_QDrone_QGC_OF_RYPI_FA';mission='Mission_Server_QDrone_Manual_Flight_FrSky_FA';
load_system(fullfile(project,[m '.mdl']));
assert(strcmp(get_param(m,'SimulationStatus'),'stopped'),'Stop Stabilizer first.');
assert(~bdIsLoaded(mission)||strcmp(get_param(mission,'SimulationStatus'),'stopped'), ...
    'Stop Mission Server first.');
assert(strcmp(get_param(m,'Dirty'),'off'),'Save pending Stabilizer changes first.');
s=[m '/STABILIZER - QDRONE'];viewer=[s '/Optical Flow Monitor/Optical Flow Camera feed'];
logger=[s '/FLIGHT SAFETY/FA Timing Diagnostic Log'];
assert(getSimulinkBlockHandle(viewer)~=-1&&getSimulinkBlockHandle(logger)~=-1, ...
    'Require the compressed preview and timing diagnostics blocks.');
assert(strcmp(get_param(viewer,'BlockType'),'SubSystem'),'Unexpected preview block.');
backup=fullfile(project,'_archive','fa_video_trial');
if ~isfolder(backup),mkdir(backup);end
copyfile(get_param(m,'FileName'),fullfile(backup,[m '_' datestr(now,'yyyymmdd_HHMMSS') '.mdl']));
if videoEnabled,commented='off';label='VideoOn';
else,commented='on';label='NoVideo';end
set_param(viewer,'Commented',commented);
set_param(logger,'file_name',['saved_data/%m/TimingDiagnostics_' label '_%{char(datetime)}.mat']);
set_param(m,'DownloadToTarget','off');set_param(m,'SimulationCommand','update');save_system(m);
fprintf('PASS: %s timing trial. Camera and flow remain enabled.\n',label);
if buildExecutable
    slbuild(m);
    fprintf('PASS: video timing trial local build. Download, Start and Arm remain explicit.\n');
end
end
