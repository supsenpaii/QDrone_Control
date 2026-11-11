function model=setup_qgc_optical_flow(buildExecutable)
% Add the existing camera/flow monitor to the saved QGC controller copy.
% Never download/start/arm. Does not replace gains with those in the OF example.
if nargin<1,buildExecutable=false;end
folder=fileparts(mfilename('fullpath'));project=fileparts(folder);
cd(project);addpath(folder,fullfile(project,'optical_flow'));
source='Stabilizer_QDrone_QGC';model='Stabilizer_QDrone_QGC_OF';
flowSource='Stabilizer_QDrone_048378_OF_Ready';
assert(~bdIsLoaded(model),'Close %s before setup.',model);
assert(~bdIsLoaded(source) || strcmp(get_param(source,'Dirty'),'off'), ...
    'Save the current QGC model first; setup copies the saved file.');
assert(~bdIsLoaded(flowSource) || strcmp(get_param(flowSource,'Dirty'),'off'), ...
    'Save or close the optical flow reference first.');
pending=fullfile(folder,'pending');if ~exist(pending,'dir'),mkdir(pending);end
output=fullfile(project,[model '.mdl']);
f=fopen(fullfile(project,[source '.mdl']),'r','n','windows-1252');assert(f~=-1,'QGC source file missing.');
body=fread(f,Inf,'*char')';fclose(f);body=strrep(body,source,model);
pendingFile=fullfile(pending,[model '.mdl']);
f=fopen(pendingFile,'w','n','windows-1252');assert(f~=-1);fwrite(f,body,'char');fclose(f);
load_system(pendingFile);cleanup=onCleanup(@()close_system(model,0)); %#ok<NASGU>
original=connections(model);
referenceWasLoaded=bdIsLoaded(flowSource);
load_system(fullfile(project,[flowSource '.slx']));
ctrl=[model '/STABILIZER - QDRONE'];
monitor=[ctrl '/Optical Flow Monitor'];
add_block([flowSource '/STABILIZER - QDRONE/Optical Flow Monitor'],monitor, ...
    'Position',[2200 650 2500 850]);
if ~referenceWasLoaded,close_system(flowSource,0);end
connect(ctrl,[ctrl '/' sprintf('Select Angular\nSpeed')],1,monitor,1);
connect(ctrl,[ctrl '/Select Angular Position'],1,monitor,2);
% Copied display blocks retain tags containing the original model path.
viewer=[monitor '/Optical Flow Camera feed/Video Display'];
if getSimulinkBlockHandle(viewer)~=-1,set_param(viewer,'Tag',viewer);end
oldInit=get_param(model,'InitFcn');
set_param(model,'InitFcn',sprintf('addpath(''%s'');\n%s', ...
    strrep(fullfile(project,'optical_flow'),'''',''''''),oldInit));
% Append the eight *pixel* flow fields after the existing 59 doubles.
tag='QGC_OPTICAL_FLOW_STATUS';
sink=[monitor '/QGC Flow Tap'];
add_block('simulink/Signal Routing/Goto',sink,'GotoTag',tag, ...
    'TagVisibility','global','Position',[1100 450 1270 480]);
% Bridge 33 ms camera -> 1 ms base -> 20 ms telemetry with integer ratios.
% Avoid a direct non-integer 33:20 ms deterministic rate transition.
baseRate=[monitor '/QGC Flow Base Rate'];
add_block('simulink/Signal Attributes/Rate Transition',baseRate, ...
    'OutPortSampleTime','0.001','Position',[1020 500 1100 530]);
connect(monitor,[monitor '/Optical Flow'],1,baseRate,1);
connect(monitor,baseRate,1,sink,1);
from=[model '/QGC Flow From'];
add_block('simulink/Signal Routing/From',from,'GotoTag',tag,'Position',[1100 1700 1260 1730]);
rate=[model '/QGC Flow Rate'];
add_block('simulink/Signal Attributes/Rate Transition',rate, ...
    'OutPortSampleTime','0.02','Position',[1300 1700 1390 1730]);
connect(model,from,1,rate,1);
mux=[model '/QGC Payload'];
assert(strcmp(get_param(mux,'Inputs'),'22'),'Expected the original 59-double QGC payload.');
set_param(mux,'Inputs','23');connect(model,rate,1,mux,23);
set_param([model '/QGC Header'],'Value','[51234567;3]');
set_param([model '/QGC Packet Width 59'],'Dimensions','67');
% Keep the existing block name so the old source connection can be verified.
set_param(model,'SimulationCommand','update');
for k=1:numel(original)
    dp=get_param(original(k).dst,'PortHandles');
    line=get_param(dp.Inport(original(k).dstPort),'Line');
    assert(line~=-1,'An original connection is missing.');
    sp=get_param(line,'SrcPortHandle');
    assert(strcmp(get_param(sp,'Parent'),original(k).src) && ...
        get_param(sp,'PortNumber')==original(k).srcPort,'An original source changed.');
end
if ~exist(fullfile(project,'saved_data',model),'dir'),mkdir(fullfile(project,'saved_data',model));end
if exist(output,'file'),copyfile(output,[output '.' datestr(now,'yyyymmdd_HHMMSS') '.bak']);end
save_system(model,output);
fprintf('PASS: original controller connections retained; flow monitor added; schema=3, width=67.\n');
if buildExecutable
    set_param(model,'DownloadToTarget','off');slbuild(model);
    fprintf('PASS: local build. No download, Start or Arm.\n');
end
fprintf('Created %s. TCP telemetry port=18920. This is a flow monitor, not Position Hold.\n',output);
end

function connect(sys,source,port,destination,destPort)
sp=get_param(source,'PortHandles');dp=get_param(destination,'PortHandles');
add_line(sys,sp.Outport(port),dp.Inport(destPort),'autorouting','on');
end

function result=connections(model)
result=struct('src',{},'srcPort',{},'dst',{},'dstPort',{});
blocks=find_system(model,'FollowLinks','off','LookUnderMasks','none','Type','Block');
for k=1:numel(blocks)
    ports=get_param(blocks{k},'PortHandles');
    for j=1:numel(ports.Inport)
        line=get_param(ports.Inport(j),'Line');
        if line==-1,continue;end
        sp=get_param(line,'SrcPortHandle');
        result(end+1)=struct('src',get_param(sp,'Parent'),'srcPort',get_param(sp,'PortNumber'), ...
            'dst',blocks{k},'dstPort',j); %#ok<AGROW>
    end
end
end
