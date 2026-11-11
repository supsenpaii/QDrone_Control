function setup_fa_timing_diagnostics(buildExecutable)
% Log elapsed task time and task CPU time together with the sample interval.
% timing_diagnostics rows: simulation time, sample interval, elapsed, CPU.
if nargin<1,buildExecutable=true;end
folder=fileparts(mfilename('fullpath'));project=fileparts(folder);cd(project);addpath(folder);
m='Stabilizer_QDrone_QGC_OF_RYPI_FA';mission='Mission_Server_QDrone_Manual_Flight_FrSky_FA';
load_system(fullfile(project,[m '.mdl']));
assert(strcmp(get_param(m,'SimulationStatus'),'stopped'),'Stop Stabilizer first.');
assert(~bdIsLoaded(mission)||strcmp(get_param(mission,'SimulationStatus'),'stopped'), ...
    'Stop Mission Server first.');
assert(strcmp(get_param(m,'Dirty'),'off'),'Save pending Stabilizer changes first.');
s=[m '/STABILIZER - QDRONE'];safety=[s '/FLIGHT SAFETY'];
assert(strcmp(get_param([safety '/Computation Time'],'use_model'),'on') ...
    &&strcmp(get_param([safety '/Computation Time'],'use_cpu_time'),'off'), ...
    'Expected the existing elapsed task-time measurement.');
backup=fullfile(project,'_archive','fa_timing_diagnostics');
if ~isfolder(backup),mkdir(backup);end
copyfile(get_param(m,'FileName'),fullfile(backup,[m '_' datestr(now,'yyyymmdd_HHMMSS') '.mdl']));
cpu=[safety '/FA CPU Time'];
if getSimulinkBlockHandle(cpu)==-1
    add_block([safety '/Computation Time'],cpu,'Position',[210 470 310 510]);
end
set_param(cpu,'use_model','on','use_cpu_time','on','sample_time','qc_get_step_size');
mux=[safety '/FA Timing Diagnostic Mux'];
if getSimulinkBlockHandle(mux)==-1
    add_block('simulink/Signal Routing/Mux',mux,'Inputs','3','Position',[410 440 420 550]);
end
logger=[safety '/FA Timing Diagnostic Log'];
if getSimulinkBlockHandle(logger)==-1
    add_block([s '/To Host File'],logger,'Position',[500 455 675 535]);
end
set_param(logger,'file_name','saved_data/%m/TimingDiagnostics_%{char(datetime)}.mat', ...
    'variable_name','timing_diagnostics','decimation','1','sample_time','0.001', ...
    'file_format','MAT-file v.5','options','No header or footer','append','off');
wire('Sample Time',1,'FA Timing Diagnostic Mux',1);
wire('Computation Time',1,'FA Timing Diagnostic Mux',2);
wire('FA CPU Time',1,'FA Timing Diagnostic Mux',3);
wire('FA Timing Diagnostic Mux',1,'FA Timing Diagnostic Log',1);
set_param(m,'DownloadToTarget','off');set_param(m,'SimulationCommand','update');save_system(m);
fprintf('PASS: sample interval, elapsed task time and CPU time diagnostic log.\n');
if buildExecutable
    slbuild(m);
    fprintf('PASS: timing diagnostics local build. Download, Start and Arm remain explicit.\n');
end
    function wire(src,sp,dst,dp)
        a=get_param([safety '/' src],'PortHandles');b=get_param([safety '/' dst],'PortHandles');
        line=get_param(b.Inport(dp),'Line');
        if line==-1
            add_line(safety,a.Outport(sp),b.Inport(dp),'autorouting','on');
        else
            assert(get_param(line,'SrcPortHandle')==a.Outport(sp),'Unexpected diagnostic input.');
        end
    end
end
