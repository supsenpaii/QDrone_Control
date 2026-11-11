function fix_sb_idle_link(buildExecutable)
% Qualify SB data with Host Initialize connection state and an observed sample.
% Host Game Controller/new is an event, not a periodic heartbeat.
if nargin<1,buildExecutable=false;end
folder=fileparts(mfilename('fullpath'));project=fileparts(folder);
cd(project);addpath(folder);
mission='Mission_Server_QDrone_Manual_Flight_FrSky_FA';
stabilizer='Stabilizer_QDrone_QGC_OF_RYPI_FA';
load_system(fullfile(project,[mission '.mdl']));
assert(strcmp(get_param(mission,'SimulationStatus'),'stopped'),'Stop Mission Server FA first.');
assert(~bdIsLoaded(stabilizer)||strcmp(get_param(stabilizer,'SimulationStatus'),'stopped'),'Stop Stabilizer first.');
assert(strcmp(get_param(mission,'Dirty'),'off'),'Save pending Mission changes first.');
backup=fullfile(project,'_archive','sb_idle_link');
if ~isfolder(backup),mkdir(backup);end
copyfile(get_param(mission,'FileName'),fullfile(backup, ...
    [mission '_' datestr(now,'yyyymmdd_HHMMSS') '.mdl']));
js=[mission '/MISSION SERVER QDRONE MANUAL FLIGHT/JOYSTICK'];
tap=[js '/FA SB Fresh Tap'];
% Permit rerunning the installer, but reject an unexpected previous source.
ports=get_param(tap,'PortHandles');line=get_param(ports.Inport(1),'Line');
assert(line~=-1,'Missing SB validity connection.');
source=get_param(line,'SrcPortHandle');parent=get_param(source,'Parent');
assert(any(strcmp(parent,{[js '/Host Game Controller'],[js '/FA SB Sample Valid']})), ...
    'Unexpected source for SB validity.');
add('Compare To Constant','FA SB Host Connected',[20 1020 140 1055], ...
    'const','2','relop','==');
add('Compare To Constant','FA SB Host No Error',[20 1090 140 1125], ...
    'const','0','relop','==');
add('Logical Operator','FA SB Host Link',[210 1020 270 1100],'Operator','AND','Inputs','2');
add('Logical Operator','FA SB New Or Seen',[210 1160 270 1230],'Operator','OR','Inputs','2');
add('Logical Operator','FA SB Sample Valid',[350 1070 420 1140],'Operator','AND','Inputs','2');
block=[js '/FA SB Seen Memory'];
if getSimulinkBlockHandle(block)==-1
    add_block('simulink/Discrete/Unit Delay',block,'Position',[490 1190 560 1230], ...
        'SampleTime','0.001','InitialCondition','0');
end
block=[js '/FA SB Link Ready'];
if getSimulinkBlockHandle(block)==-1
    add_block('simulink/Sinks/Display',block,'Position',[490 1060 600 1100],'Decimation','1');
end
wire('Host Initialize',1,'FA SB Host Connected',1);
wire('Host Initialize',2,'FA SB Host No Error',1);
wire('FA SB Host Connected',1,'FA SB Host Link',1);
wire('FA SB Host No Error',1,'FA SB Host Link',2);
wire('Host Game Controller',1,'FA SB New Or Seen',1);
wire('FA SB Seen Memory',1,'FA SB New Or Seen',2);
wire('FA SB Host Link',1,'FA SB Sample Valid',1);
wire('FA SB New Or Seen',1,'FA SB Sample Valid',2);
wire('FA SB Sample Valid',1,'FA SB Seen Memory',1);
wire('FA SB Sample Valid',1,'FA SB Link Ready',1);
if strcmp(parent,[js '/Host Game Controller'])
    % Remove only this tap, retaining the existing Pulse Check branch.
    delete_line(js,'Host Game Controller/1','FA SB Fresh Tap/1');
end
wire('FA SB Sample Valid',1,'FA SB Fresh Tap',1);
set_param(mission,'DownloadToTarget','off');
set_param(mission,'SimulationCommand','update');save_system(mission);
fprintf('PASS: SB link qualification installed. Idle holds data while connected; disconnect clears sample readiness.\n');
if buildExecutable
    slbuild(mission);
    fprintf('PASS: Mission local build. Download and Start remain explicit.\n');
end
open_system(js);
    function add(type,name,position,varargin)
        block=[js '/' name];
        if getSimulinkBlockHandle(block)==-1
            add_block(['simulink/Logic and Bit Operations/' type],block, ...
                'Position',position,varargin{:});
        end
    end
    function wire(src,sp,dst,dp)
        a=get_param([js '/' src],'PortHandles');b=get_param([js '/' dst],'PortHandles');
        existing=get_param(b.Inport(dp),'Line');
        if existing==-1
            add_line(js,a.Outport(sp),b.Inport(dp),'autorouting','on');
        else
            assert(get_param(existing,'SrcPortHandle')==a.Outport(sp),'Unexpected input to %s.',dst);
        end
    end
end
