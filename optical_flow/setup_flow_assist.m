function [model,mission]=setup_flow_assist(buildExecutable)
% Separate paired models; no download, Start, Arm or change to original files.
if nargin<1,buildExecutable=false;end
folder=fileparts(mfilename('fullpath'));project=fileparts(folder);cd(project);
addpath(folder,fullfile(project,'pid_diagnostics'),fullfile(project,'qgc_bridge'));
test_flow_assist;
source='Stabilizer_QDrone_QGC_OF_RYPI';model=[source '_FA'];
missionSource='Mission_Server_QDrone_Manual_Flight_FrSky';mission=[missionSource '_FA'];
pending=fullfile(folder,'pending');if ~isfolder(pending),mkdir(pending);end
for pair={ {source,model},{missionSource,mission} }
    item=pair{1};assert(~bdIsLoaded(item{2}),'Close %s before setup.',item{2});
    assert(~bdIsLoaded(item{1})||strcmp(get_param(item{1},'Dirty'),'off'), ...
        'Save source %s first; setup copies the saved file.',item{1});
end
clone(source,model,project,pending);cleanup=onCleanup(@()close_system(model,0)); %#ok<NASGU>
s=[model '/STABILIZER - QDRONE'];monitor=[s '/Optical Flow Monitor'];
before=connections(model);[cfg,R]=flow_assist_config;
assert(cfg(1)==0,'First setup requires calibration lock=0.');
assert(strcmp(get_param(model,'FixedStep'),'0.001'));
modeBlock=[s '/Mode (0,1)'];modePorts=get_param(modeBlock,'PortHandles');
modeDests=get_param(get_param(modePorts.Outport(1),'Line'),'DstPortHandle');
assert(all(modeDests~=-1));delete_line(get_param(modePorts.Outport(1),'Line'));
decoder=[s '/FA Mode Decoder'];
add_block('simulink/User-Defined Functions/MATLAB Function',decoder,'Position',[-500 1100 -320 1180]);
chart(decoder,sprintf(['function [base,request]=decode(raw)\n%%#codegen\n' ...
    'base=double(raw==1);request=double(raw==2);\nend\n']), ...
    {'raw','base','request'},{'1','1','1'});
wire(s,modeBlock,1,decoder,1);dp=get_param(decoder,'PortHandles');
for i=1:numel(modeDests),add_line(s,dp.Outport(1),modeDests(i),'autorouting','on');end

% Atomic frame packet through ONE rate transition; do not join independent
% frame time/flow transitions that can pair data from different frames.
packet=[monitor '/FA Frame Packet'];
add_block('simulink/Signal Routing/Mux',packet,'Inputs','2','Position',[1100 600 1110 665]);
wire(monitor,[monitor '/Frame Clock'],1,packet,1);wire(monitor,[monitor '/Optical Flow'],1,packet,2);
rate=[monitor '/FA Frame Base Rate'];
add_block('simulink/Signal Attributes/Rate Transition',rate,'OutPortSampleTime','0.001', ...
    'Position',[1150 600 1250 650]);wire(monitor,packet,1,rate,1);
tap=[monitor '/FA Frame Tap'];add_block('simulink/Signal Routing/Goto',tap, ...
    'GotoTag','FA_FRAME_PACKET','TagVisibility','global','Position',[1300 600 1410 630]);
wire(monitor,rate,1,tap,1);
from=[s '/FA Frame From'];add_block('simulink/Signal Routing/From',from,'GotoTag','FA_FRAME_PACKET', ...
    'Position',[1400 3000 1550 3030]);
split=[s '/FA Frame Split'];add_block('simulink/Signal Routing/Demux',split,'Outputs','[1 8]', ...
    'Position',[1600 3000 1610 3060]);wire(s,from,1,split,1);

constant(s,'FA Configuration',mat2str(cfg,17),[1300 3300 1500 3340]);
constant(s,'FA Camera To Body',mat2str(R,17),[1300 3360 1500 3400]);
fa=[s '/Flow Assist'];
add_block('simulink/User-Defined Functions/MATLAB Function',fa,'Position',[1800 3000 2080 3350]);
chart(fa,sprintf(['function [out,diagnostic]=assist(command,flow,frameTime,gyro,attitude,arm,throttle,mode,request,config,rotation,time)\n%%#codegen\n' ...
    '[out,diagnostic]=qdrone_fa_step(command,flow,frameTime,gyro,attitude,arm,throttle,mode,request,config,rotation,time);\nend\n']), ...
    {'command','flow','frameTime','gyro','attitude','arm','throttle','mode','request','config','rotation','time','out','diagnostic'}, ...
    {'[3 1]','[8 1]','1','[3 1]','[3 1]','1','1','1','1','[13 1]','[3 3]','1','[3 1]','[16 1]'});
sat=[s '/' sprintf('Saturate\nStabilization\nCommand')];sp=get_param(sat,'PortHandles');
ln=get_param(sp.Inport(1),'Line');cmdSource=get_param(ln,'SrcPortHandle');
cmdParent=get_param(cmdSource,'Parent');cmdPort=get_param(cmdSource,'PortNumber');
% Delete exactly this branch; retain all other consumers of raw pilot command.
delete_line(s,[esc(get_param(cmdParent,'Name')) '/' num2str(cmdPort)], ...
    [esc(get_param(sat,'Name')) '/1']);
reshapeWire(s,cmdParent,cmdPort,fa,1,3,'FA Pilot Column');
faOutput=[s '/FA Command Vector'];
add_block('simulink/Math Operations/Reshape',faOutput,'OutputDimensionality','1-D array', ...
    'Position',[2120 3000 2210 3040]);wire(s,fa,1,faOutput,1);wire(s,faOutput,1,sat,1);
reshapeWire(s,split,2,fa,2,8,'FA Flow Column');wire(s,split,1,fa,3);
reshapeWire(s,[s '/' sprintf('Select Angular\nSpeed')],1,fa,4,3,'FA Gyro Column');
reshapeWire(s,[s '/Select Angular Position'],1,fa,5,3,'FA Attitude Column');
wire(s,[s '/RYPI Actual Arm'],1,fa,6);wire(s,[s '/Throttle (N)'],1,fa,7);
wire(s,decoder,1,fa,8);wire(s,decoder,2,fa,9);
wire(s,[s '/FA Configuration'],1,fa,10);wire(s,[s '/FA Camera To Body'],1,fa,11);
wire(s,[s '/RYPI Time'],1,fa,12);
add_block('simulink/Sinks/Display',[s '/FA Diagnostic Status First'], ...
    'Position',[2100 3400 2300 3800]);wire(s,fa,2,[s '/FA Diagnostic Status First'],1);

% Diagnostic payload: time, raw command, output command, frame(9), gyro(3),
% attitude(3), Arm, throttle, baseMode, request, diagnostic(16), config(13), R(9).
logMux=[s '/FA Log Mux'];add_block('simulink/Signal Routing/Mux',logMux,'Inputs','14', ...
    'Position',[2300 3000 2310 3470]);
sources={'RYPI Time','FA Pilot Column','Flow Assist','FA Frame From','FA Gyro Column','FA Attitude Column', ...
    'RYPI Actual Arm','Throttle (N)','FA Mode Decoder','FA Mode Decoder','Flow Assist','FA Configuration','FA Rotation Column','FA Raw Mode Column'};
reshapeBlock(s,[s '/FA Camera To Body'],1,9,'FA Rotation Column');
reshapeBlock(s,modeBlock,1,1,'FA Raw Mode Column');
for i=1:numel(sources)
    port=1;if i==10||i==11,port=2;end
    wire(s,[s '/' sources{i}],port,logMux,i);
end
width=[s '/FA Log Width'];add_block('simulink/Signal Attributes/Signal Specification',width, ...
    'Dimensions','65','Position',[2360 3100 2430 3140]);wire(s,logMux,1,width,1);
logger=[s '/FA Log'];add_block([s '/To Host File'],logger,'Position',[2500 3100 2690 3170]);
set_param(logger,'file_name','saved_data/%m/FlowAssist_%{char(datetime)}.mat', ...
    'variable_name','flow_assist_data','decimation','2','sample_time','0.001');wire(s,width,1,logger,1);
oldInit=get_param(model,'InitFcn');set_param(model,'InitFcn',sprintf('addpath(''%s'');\n%s',strrep(folder,'''',''''''),oldInit));
set_param(model,'DownloadToTarget','off');set_param(model,'SimulationCommand','update');
for i=1:numel(before)
    expected=before(i);
    if strcmp(expected.dst,sat)&&expected.dstPort==1,expected.src=faOutput;expected.srcPort=1;end
    if strcmp(expected.src,modeBlock),expected.src=decoder;expected.srcPort=1;end
    verify(expected);
end
assert(strcmp(get_param([s '/FA Configuration'],'Value'),mat2str(cfg,17)));

% Separate paired Mission Server. No guessed RC assignment: button index 0
% disables selection. Existing Arm/E-stop and base Angle/Torque remain wired.
clone(missionSource,mission,project,pending);cleanMission=onCleanup(@()close_system(mission,0)); %#ok<NASGU>
ms=[mission '/MISSION SERVER QDRONE MANUAL FLIGHT'];js=[ms '/JOYSTICK'];
msBefore=connections(mission);
tap=[js '/FA Buttons Tap'];add_block('simulink/Signal Routing/Goto',tap,'GotoTag','FA_RC_BUTTONS', ...
    'TagVisibility','global','Position',[-200 750 -50 790]);
wire(js,[js '/Host Game Controller'],2,tap,1);
from=[ms '/FA Buttons'];add_block('simulink/Signal Routing/From',from,'GotoTag','FA_RC_BUTTONS', ...
    'Position',[20 750 180 790]);
add_block('simulink/Sinks/Display',[ms '/FA Raw Buttons 1 to 32'], ...
    'Position',[20 950 220 1400]);wire(ms,from,1,[ms '/FA Raw Buttons 1 to 32'],1);
constant(ms,'FA Button Index','0',[20 810 180 850]);
selector=[ms '/FA Switch Selection'];add_block('simulink/User-Defined Functions/MATLAB Function',selector, ...
    'Position',[250 740 440 850]);
chart(selector,sprintf(['function encoded=select(buttons,index,base)\n%%#codegen\n' ...
    'encoded=base;\nif base==0 && isfinite(index) && index>=3 && index<=32 && index==floor(index)\n' ...
    '    if isfinite(buttons(index)) && buttons(index)>0.5,encoded=2;end\nend\nend\n']), ...
    {'buttons','index','base','encoded'},{'[32 1]','1','1','1'});
reshapeWire(ms,from,1,selector,1,32,'FA Buttons Column');
wire(ms,[ms '/FA Button Index'],1,selector,2);
base=[ms '/' sprintf('Manual Flight Mode\n1 - Torque Mode,   0 - Angle Mode')];
wire(ms,base,1,selector,3);
destination=[ms '/Mode'];pp=get_param(destination,'PortHandles');l=get_param(pp.Inport(1),'Line');
assert(strcmp(get_param(get_param(l,'SrcPortHandle'),'Parent'),base));
delete_line(ms,[esc(get_param(base,'Name')) '/1'],'Mode/1');wire(ms,selector,1,destination,1);
add_block('simulink/Sinks/Display',[ms '/FA Encoded Mode 0 Angle 1 Torque 2 Assist'], ...
    'Position',[500 750 740 800]);wire(ms,selector,1,[ms '/FA Encoded Mode 0 Angle 1 Torque 2 Assist'],1);
set_param(mission,'DownloadToTarget','off');set_param(mission,'SimulationCommand','update');
for i=1:numel(msBefore)
    expected=msBefore(i);
    if strcmp(expected.dst,destination)&&expected.dstPort==1,expected.src=selector;expected.srcPort=1;end
    verify(expected);
end
for name={model,mission}
    output=fullfile(project,[name{1} '.mdl']);
    if isfile(output),copyfile(output,[output '.' datestr(now,'yyyymmdd_HHMMSS') '.bak']);end
    if ~isfolder(fullfile(project,'saved_data',name{1})),mkdir(fullfile(project,'saved_data',name{1}));end
    save_system(name{1},output);
end
% User confirmed SB is Rz /9: up=-1, middle=0, down=+1.
setup_sb_flow_assist(false);
fix_fa_frame_freshness(false);
fprintf('PASS: paired FA diagrams; log width=65; calibration lock=0; SB /9 mode selector installed.\n');
if buildExecutable
    slbuild(model);slbuild(mission);
    fprintf('PASS: local builds. No download, Start or Arm.\n');
end
fprintf('Created %s and %s. Read optical_flow/FLOW_ASSIST_VI.md.\n',model,mission);
end

function clone(source,dest,project,pending)
f=fopen(fullfile(project,[source '.mdl']),'r','n','windows-1252');assert(f~=-1,'Source missing: %s',source);
b=fread(f,Inf,'*char')';fclose(f);b=strrep(b,source,dest);
p=fullfile(pending,[dest '.mdl']);f=fopen(p,'w','n','windows-1252');assert(f~=-1);fwrite(f,b,'char');fclose(f);load_system(p);
end
function chart(path,script,names,sizes)
r=sfroot;c=r.find('-isa','Stateflow.EMChart','Path',path);assert(numel(c)==1);c.Script=script;
c.ChartUpdate='DISCRETE';c.SampleTime='0.001';
for i=1:numel(names),d=c.find('-isa','Stateflow.Data','Name',names{i});assert(numel(d)==1);d.Props.Array.Size=sizes{i};end
end
function constant(s,name,value,pos)
add_block('simulink/Sources/Constant',[s '/' name],'Value',value,'Position',pos);
end
function wire(s,src,sp,dst,dp)
a=get_param(src,'PortHandles');b=get_param(dst,'PortHandles');add_line(s,a.Outport(sp),b.Inport(dp),'autorouting','on');
end
function reshapeWire(s,src,port,dest,in,width,name)
reshapeBlock(s,src,port,width,name);wire(s,[s '/' name],1,dest,in);
end
function reshapeBlock(s,src,port,width,name)
add_block('simulink/Math Operations/Reshape',[s '/' name],'OutputDimensionality','Customize', ...
    'OutputDimensions',sprintf('[%d 1]',width),'Position',[1200 3500+port*45 1300 3530+port*45]);
wire(s,src,port,[s '/' name],1);
end
function value=esc(value),value=strrep(value,'/','//');end
function verify(x)
p=get_param(x.dst,'PortHandles');ln=get_param(p.Inport(x.dstPort),'Line');assert(ln~=-1,'Connection missing: %s',x.dst);
sp=get_param(ln,'SrcPortHandle');assert(strcmp(get_param(sp,'Parent'),x.src)&&get_param(sp,'PortNumber')==x.srcPort, ...
    'Original connection changed unexpectedly: %s',x.dst);
end
function r=connections(model)
r=struct('src',{},'srcPort',{},'dst',{},'dstPort',{});
blocks=find_system(model,'FollowLinks','off','LookUnderMasks','none','Type','Block');
for i=1:numel(blocks)
    p=get_param(blocks{i},'PortHandles');
    for j=1:numel(p.Inport)
        l=get_param(p.Inport(j),'Line');if l==-1,continue;end
        sp=get_param(l,'SrcPortHandle');if sp==-1,continue;end
        r(end+1)=struct('src',get_param(sp,'Parent'),'srcPort',get_param(sp,'PortNumber'),'dst',blocks{i},'dstPort',j); %#ok<AGROW>
    end
end
end
