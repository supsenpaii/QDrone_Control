function setup_fa_codegen_diagnostics(buildExecutable)
% Retain block outputs between generated output-function invocations.
% This avoids local output temporaries shared across separate MajorTimeStep
% branches, where GCC reports potentially uninitialized Stream/Threshold data.
if nargin<1,buildExecutable=true;end
folder=fileparts(mfilename('fullpath'));project=fileparts(folder);cd(project);addpath(folder);
m='Stabilizer_QDrone_QGC_OF_RYPI_FA';
mission='Mission_Server_QDrone_Manual_Flight_FrSky_FA';
load_system(fullfile(project,[m '.mdl']));
assert(strcmp(get_param(m,'SimulationStatus'),'stopped'),'Stop Stabilizer first.');
assert(~bdIsLoaded(mission)||strcmp(get_param(mission,'SimulationStatus'),'stopped'), ...
    'Stop Mission Server first.');
assert(strcmp(get_param(m,'Dirty'),'off'),'Save pending Stabilizer changes first.');
backup=fullfile(project,'_archive','fa_codegen_diagnostics');
if ~isfolder(backup),mkdir(backup);end
copyfile(get_param(m,'FileName'),fullfile(backup,[m '_' datestr(now,'yyyymmdd_HHMMSS') '.mdl']));
set_param(m,'LocalBlockOutputs','off','DownloadToTarget','off');
set_param(m,'SimulationCommand','update');save_system(m);
fprintf('PASS: block outputs retained in generated model storage.\n');
if buildExecutable
    slbuild(m);
    fprintf('PASS: local build. Inspect compiler output for remaining warnings.\n');
end
end
