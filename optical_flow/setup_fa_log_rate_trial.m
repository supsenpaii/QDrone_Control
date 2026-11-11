function setup_fa_log_rate_trial(buildExecutable,lowLogRate)
% Select 100 Hz numeric logs or restore the measured NoVideo baseline rates.
% Keep the task CPU/elapsed/sample interval diagnostic at 1 kHz.
if nargin<1,buildExecutable=true;end
if nargin<2,lowLogRate=true;end
validateattributes(lowLogRate,{'logical','numeric'},{'scalar','binary'});
folder=fileparts(mfilename('fullpath'));project=fileparts(folder);cd(project);addpath(folder);
m='Stabilizer_QDrone_QGC_OF_RYPI_FA';mission='Mission_Server_QDrone_Manual_Flight_FrSky_FA';
load_system(fullfile(project,[m '.mdl']));
assert(strcmp(get_param(m,'SimulationStatus'),'stopped'),'Stop Stabilizer first.');
assert(~bdIsLoaded(mission)||strcmp(get_param(mission,'SimulationStatus'),'stopped'), ...
    'Stop Mission Server first.');
assert(strcmp(get_param(m,'Dirty'),'off'),'Save pending Stabilizer changes first.');
s=[m '/STABILIZER - QDRONE'];
assert(strcmp(get_param([s '/Optical Flow Monitor/Optical Flow Camera feed'],'Commented'),'on'), ...
    'First apply and record the NoVideo timing trial.');
diagnostic=[s '/FLIGHT SAFETY/FA Timing Diagnostic Log'];
assert(strcmp(get_param(diagnostic,'decimation'),'1') ...
    &&strcmp(get_param(diagnostic,'sample_time'),'0.001'), ...
    'Timing diagnostic must remain at 1 kHz.');
% QUARC can resolve a saved Reference into masked/S-Function blocks, so the
% loaded SourceType is not a reliable identifier. Inspect the mask parameters.
blocks=find_system(m,'LookUnderMasks','all','FollowLinks','on','Type','Block');
loggerVariables=repmat({''},size(blocks));
for j=1:numel(blocks)
    parameters=get_param(blocks{j},'DialogParameters');
    if isstruct(parameters)&&all(isfield(parameters,{'variable_name','decimation','sample_time'}))
        loggerVariables{j}=get_param(blocks{j},'variable_name');
    end
end
variables={'QDrone_data','stabilizer_data','flow_assist_data','roll_yaw_pi_data'};
selected=cell(size(variables));
for k=1:numel(variables)
    matches=find(strcmp(loggerVariables,variables{k}));
    assert(numel(matches)==1,'Expected one logger for %s, found %d. Available variables: %s', ...
        variables{k},numel(matches),strjoin(loggerVariables(~cellfun('isempty',loggerVariables)),', '));
    selected{k}=blocks{matches};
    assert(any(strcmp(get_param(selected{k},'sample_time'),{'-1','0.001'})), ...
        'Unexpected sample time for %s.',variables{k});
end
backup=fullfile(project,'_archive','fa_log_rate_trial');
if ~isfolder(backup),mkdir(backup);end
copyfile(get_param(m,'FileName'),fullfile(backup,[m '_' datestr(now,'yyyymmdd_HHMMSS') '.mdl']));
if lowLogRate
    decimation=[10 10 10 10];label='NoVideo_LowLog';
else
    decimation=[1 1 2 2];label='NoVideo_Repeat';
end
for k=1:numel(selected)
    set_param(selected{k},'decimation',num2str(decimation(k)));
    fprintf('%s log decimation %d\n',variables{k},decimation(k));
end
set_param(diagnostic,'file_name',['saved_data/%m/TimingDiagnostics_' label '_%{char(datetime)}.mat']);
set_param(m,'DownloadToTarget','off');set_param(m,'SimulationCommand','update');save_system(m);
fprintf('PASS: %s numeric-log profile, timing diagnostics at 1 kHz.\n',label);
if buildExecutable
    slbuild(m);
    fprintf('PASS: log-rate timing trial local build. Download, Start and Arm remain explicit.\n');
end
end
