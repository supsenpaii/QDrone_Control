function mission=setup_sb_flow_assist(buildExecutable)
% Upgrade EXISTING FA Mission Server: confirmed SB is Host Game Controller/9,
% UP=-1, MIDDLE=0, DOWN=+1. No download, Start, Arm or calibration changes.
if nargin<1,buildExecutable=false;end
folder=fileparts(mfilename('fullpath'));project=fileparts(folder);cd(project);addpath(folder);
test_sb_mode;
mission='Mission_Server_QDrone_Manual_Flight_FrSky_FA';stabilizer='Stabilizer_QDrone_QGC_OF_RYPI_FA';
load_system(fullfile(project,[mission '.mdl']));
assert(strcmp(get_param(mission,'SimulationStatus'),'stopped'),'Disarm and Stop Mission Server first.');
assert(~bdIsLoaded(stabilizer)||strcmp(get_param(stabilizer,'SimulationStatus'),'stopped'),'Stop Stabilizer first.');
assert(strcmp(get_param(mission,'Dirty'),'off'),'Save Mission Server first; installer backs up the saved file.');
file=get_param(mission,'FileName');copyfile(file,[file '.' datestr(now,'yyyymmdd_HHMMSS') '.sb.bak']);
ms=[mission '/MISSION SERVER QDRONE MANUAL FLIGHT'];js=[ms '/JOYSTICK'];
selector=[ms '/FA Switch Selection'];assert(getSimulinkBlockHandle(selector)~=-1,'FA selector missing.');
before=connections(mission);
% Replacing a chart regenerates its internal S-function lines. Audit its
% external connections and all other blocks, not generated chart internals.
keep=true(size(before));prefix=[selector '/'];
for i=1:numel(before)
    keep(i)=~strncmp(before(i).src,prefix,numel(prefix))&&~strncmp(before(i).dst,prefix,numel(prefix));
end
before=before(keep);
% Check the old Rz path rather than assuming it is an unused control input.
% Current source labels Rz as Takeoff/Autoland, but that flag is trigger[2].
% Only trigger[1] (Arm) and [3] (Emergency) feed these command outputs.
assert(strcmp(get_param([ms '/Selector'],'Indices'),'[1]')&& ...
    strcmp(get_param([ms '/Selector1'],'Indices'),'[3]'), ...
    'Unexpected trigger mapping; audit existing Rz/Takeoff usage before installing SB.');
base=[ms '/' sprintf('Manual Flight Mode\n1 - Torque Mode,   0 - Angle Mode')];
assert(str2double(get_param(base,'Value'))==0,'Keep the existing base mode at Angle (0).');
% Do not let button and axis selectors compete for the mode.
set_param([ms '/FA Button Index'],'Value','0');
if getSimulinkBlockHandle([js '/FA SB Rz Tap'])==-1
    add_block('simulink/Signal Routing/Goto',[js '/FA SB Rz Tap'],'GotoTag','FA_SB_RZ', ...
        'TagVisibility','global','Position',[-200 820 -50 850]);
    wire(js,[js '/Host Game Controller'],9,[js '/FA SB Rz Tap'],1);
    add_block('simulink/Signal Routing/Goto',[js '/FA SB Fresh Tap'],'GotoTag','FA_SB_FRESH', ...
        'TagVisibility','global','Position',[-200 880 -50 910]);
    wire(js,[js '/Host Game Controller'],1,[js '/FA SB Fresh Tap'],1);
    add_block('simulink/Signal Routing/From',[ms '/FA SB Rz'],'GotoTag','FA_SB_RZ','Position',[20 1450 170 1480]);
    add_block('simulink/Signal Routing/From',[ms '/FA SB Fresh'],'GotoTag','FA_SB_FRESH','Position',[20 1500 170 1530]);
    add_block('simulink/Sources/Digital Clock',[ms '/FA SB Time'],'sampleTime','0.001','Position',[20 1550 170 1580]);
    add_block('simulink/Sources/Constant',[ms '/FA SB Enabled'],'Value','1','Position',[20 1600 170 1630]);
    add_block('simulink/Sinks/Display',[ms '/FA SB Raw -1 Up 0 Middle 1 Down'],'Position',[250 1450 490 1490]);
    wire(ms,[ms '/FA SB Rz'],1,[ms '/FA SB Raw -1 Up 0 Middle 1 Down'],1);
    add_block('simulink/Sinks/Display',[ms '/FA SB Status First'],'Position',[500 1450 720 1650]);
end
r=sfroot;c=r.find('-isa','Stateflow.EMChart','Path',selector);assert(numel(c)==1);
c.Script=sprintf(['function [encoded,diagnostic]=select(buttons,index,base,sb,fresh,time,enabled)\n%%#codegen\n' ...
    '%% buttons/index retained only to preserve the original input connections.\n' ...
    '[encoded,diagnostic]=qdrone_sb_mode_step(sb,fresh,time,base,enabled);\nend\n']);
c.ChartUpdate='DISCRETE';c.SampleTime='0.001';
names={'buttons','index','base','sb','fresh','time','enabled','encoded','diagnostic'};
sizes={'[32 1]','1','1','1','1','1','1','1','[6 1]'};
for i=1:numel(names),d=c.find('-isa','Stateflow.Data','Name',names{i});assert(numel(d)==1);d.Props.Array.Size=sizes{i};end
wireMissing(ms,[ms '/FA SB Rz'],1,selector,4);
wireMissing(ms,[ms '/FA SB Fresh'],1,selector,5);
wireMissing(ms,[ms '/FA SB Time'],1,selector,6);
wireMissing(ms,[ms '/FA SB Enabled'],1,selector,7);
wireMissing(ms,selector,2,[ms '/FA SB Status First'],1);
oldInit=get_param(mission,'InitFcn');
if isempty(strfind(oldInit,folder)) %#ok<STREMP>
    set_param(mission,'InitFcn',sprintf('addpath(''%s'');\n%s',strrep(folder,'''',''''''),oldInit));
end
% Do not change base mode, Arm, E-stop, throttle, yaw or original connections.
set_param(mission,'DownloadToTarget','off');
diagFile=fullfile(folder,'sb_mode_diagnostics.log');sldiagviewer.diary(diagFile);
cleanup=onCleanup(@()sldiagviewer.diary('off')); %#ok<NASGU>
try
    set_param(mission,'SimulationCommand','update');
catch cause
    sldiagviewer.diary('off');
    fprintf(2,'%s\n',getReport(cause,'extended','hyperlinks','off'));
    if isfile(diagFile),fprintf(2,'%s\n',fileread(diagFile));end
    rethrow(cause);
end
for i=1:numel(before),verify(before(i));end
save_system(mission);
% Replace the new-data event with connection-qualified sample validity.
fix_sb_idle_link(false);
fprintf('PASS: SB diagram and original connection audit. Rz /9: -1 Angle, 0 Angle, +1 FA after UP qualification.\n');
if buildExecutable,slbuild(mission);fprintf('PASS: SB Mission Server local build. No download, Start or Arm.\n');end
open_system(ms);
hilite_system([ms '/FA SB Raw -1 Up 0 Middle 1 Down'],'find');
fprintf('On startup hold SB UP >=0.15 s, then DOWN >=0.12 s. Keep Disarmed for verification.\n');
end
function wire(s,src,sp,dst,dp)
a=get_param(src,'PortHandles');b=get_param(dst,'PortHandles');add_line(s,a.Outport(sp),b.Inport(dp),'autorouting','on');
end
function wireMissing(s,src,sp,dst,dp)
p=get_param(dst,'PortHandles');l=get_param(p.Inport(dp),'Line');
if l==-1,wire(s,src,sp,dst,dp);else
    q=get_param(l,'SrcPortHandle');assert(strcmp(get_param(q,'Parent'),src)&&get_param(q,'PortNumber')==sp,'SB connection mismatch.');
end
end
function verify(x)
p=get_param(x.dst,'PortHandles');l=get_param(p.Inport(x.dstPort),'Line');assert(l~=-1,'Missing original connection.');
q=get_param(l,'SrcPortHandle');assert(strcmp(get_param(q,'Parent'),x.src)&&get_param(q,'PortNumber')==x.srcPort, ...
    'Original connection changed: %s',x.dst);
end
function list=connections(model)
list=struct('src',{},'srcPort',{},'dst',{},'dstPort',{});
blocks=find_system(model,'FollowLinks','off','LookUnderMasks','none','Type','Block');
for i=1:numel(blocks)
    p=get_param(blocks{i},'PortHandles');
    for j=1:numel(p.Inport)
        l=get_param(p.Inport(j),'Line');if l==-1,continue;end
        q=get_param(l,'SrcPortHandle');if q==-1,continue;end
        list(end+1)=struct('src',get_param(q,'Parent'),'srcPort',get_param(q,'PortNumber'),'dst',blocks{i},'dstPort',j); %#ok<AGROW>
    end
end
end
