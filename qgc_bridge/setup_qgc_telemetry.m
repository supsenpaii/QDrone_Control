function model=setup_qgc_telemetry(source,buildExecutable)
% Passive telemetry copy of a saved Stabilizer; never download/start/arm.
% Run in MATLAB R2019a with QUARC. Existing controller connections are checked.
if nargin<1,source='Stabilizer_QDrone';end
if nargin<2,buildExecutable=false;end
folder=fileparts(mfilename('fullpath'));project=fileparts(folder);
cd(project);addpath(folder);
model=[source '_QGC']; output=fullfile(project,[model '.mdl']);
assert(~bdIsLoaded(model),'Close %s before creating the telemetry copy.',model);
assert(~bdIsLoaded(source) || strcmp(get_param(source,'Dirty'),'off'), ...
    'Save your source model first: setup uses the saved file.');
sourceFile=fullfile(project,[source '.mdl']);
% MATLAB exist(...,'file') returns 4 for Simulink models, not 2.
assert(isfile(sourceFile),'Source model missing: %s',sourceFile);
% Stage separately; only publish the final file after diagram validation.
pendingFolder=fullfile(folder,'pending');if ~exist(pendingFolder,'dir'),mkdir(pendingFolder);end
pendingFile=fullfile(pendingFolder,[model '.mdl']);
% Copy the saved MDL text: never rename or save an open source session.
f=fopen(sourceFile,'r','n','windows-1252');assert(f~=-1);
body=fread(f,Inf,'*char')';fclose(f);body=strrep(body,source,model);
f=fopen(pendingFile,'w','n','windows-1252');assert(f~=-1);fwrite(f,body,'char');fclose(f);
load_system(pendingFile);
cleanup=onCleanup(@() close_system(model,0)); %#ok<NASGU>
assert(strcmp(get_param(model,'FixedStep'),'0.001'),'Expected 1 ms controller.');
plant=[model '/PHYSICAL PLANT w// LINEAR THRUST'];
ctrl=[model '/STABILIZER - QDRONE'];
original=edges(model);
% Every original consumer must still have exactly the same source afterwards.
% 50 controller values after the 4-double framing header: see flight_schema.json.
taps={...
 plant,'Conversion',1,1; ...
 model,'PHYSICAL PLANT w/ LINEAR THRUST',1,1; ...
 model,'PHYSICAL PLANT w/ LINEAR THRUST',2,1; ...
 model,'Communication',1,1; ...
 plant,'Arm (0,1)',1,1; ...
 model,'Demux1',1,1; ...
 model,'Demux1',2,1; ...
 ctrl,'Throttle (N)',1,1; ...
 ctrl,sprintf('Saturate\nStabilization\nCommand'),1,3; ...
 model,'PHYSICAL PLANT w/ LINEAR THRUST',5,9; ...
 model,'PHYSICAL PLANT w/ LINEAR THRUST',3,6; ...
 plant,'Product1',1,4; ...
 plant,sprintf('Bias - Pulse \nOffset (%%)'),1,4; ...
 model,'STABILIZER - QDRONE',1,4; ...
 ctrl,sprintf('Saturation\nCommand \nAuthority (Nm)'),1,3; ...
 ctrl,sprintf('Saturation\nMax Rate \n(rad/s)'),1,3; ...
 ctrl,'Attitude Torques (Nm) [3]',1,3; ...
 model,'Demux1',5,3};
assert(sum(cell2mat(taps(:,4)))==50);
add_block('simulink/Sources/Constant',[model '/QGC Header'], ...
    'Value','[51234567;2]','Position',[950 700 1040 730]);
add_block('simulink/Sources/Digital Clock',[model '/QGC Time'], ...
    'sampleTime','0.02','Position',[950 750 1040 780]);
add_block('simulink/Math Operations/Gain',[model '/QGC Sequence'], ...
    'Gain','50','Position',[1080 750 1140 780]);
add_line(model,'QGC Time/1','QGC Sequence/1');
mux=[model '/QGC Payload'];
add_block('simulink/Signal Routing/Mux',mux,'Inputs',num2str(size(taps,1)+4), ...
    'Position',[1500 650 1505 1400]);
connect(model,'QGC Header',1,mux,1);
connect(model,'QGC Sequence',1,mux,2);
connect(model,'QGC Time',1,mux,3);
for k=1:size(taps,1)
    sys=taps{k,1}; name=taps{k,2};port=taps{k,3};
    % Escape literal slashes in source block names for Simulink paths.
    src=[sys '/' strrep(name,'/','//')];
    tag=sprintf('QGC_TELEMETRY_%02d',k);
    sink=[sys '/' tag];
    add_block('simulink/Signal Routing/Goto',sink,'GotoTag',tag, ...
        'TagVisibility','global','Position',[2100 1800+40*k 2220 1820+40*k]);
    sp=get_param(src,'PortHandles');gp=get_param(sink,'PortHandles');
    add_line(sys,sp.Outport(port),gp.Inport(1),'autorouting','on');
    from=[model '/' tag '_FROM'];
    add_block('simulink/Signal Routing/From',from,'GotoTag',tag, ...
        'Position',[950 810+30*k 1060 830+30*k]);
    cast=[model '/' tag '_DOUBLE'];
    add_block('simulink/Signal Attributes/Data Type Conversion',cast, ...
        'OutDataTypeStr','double','Position',[1100 810+30*k 1150 830+30*k]);
    rate=[model '/' tag '_RATE'];
    add_block('simulink/Signal Attributes/Rate Transition',rate, ...
        'OutPortSampleTime','0.02','Position',[1200 810+30*k 1260 830+30*k]);
    connect(model,[tag '_FROM'],1,cast,1);
    connect(model,[tag '_DOUBLE'],1,rate,1);
    connect(model,[tag '_RATE'],1,mux,k+3);
end
% Stabilizer has no mag/barometer HIL reads: preserve unknown fields explicitly.
add_block('simulink/Sources/Constant',[model '/QGC Unavailable Sensors'], ...
    'Value','nan(5,1)','Position',[1250 1490 1410 1520]);
connect(model,'QGC Unavailable Sensors',1,mux,size(taps,1)+4);
server=[model '/QGC Telemetry Server'];
add_block('quarc_library/Communications/Basic/Stream Server',server, ...
    'Position',[1600 950 1770 1100]);
set_param(server,'uri_source','External input port','uri','tcpip://localhost:18920', ...
    'sample_time','0.02','active','on','send_options','Send most recent data', ...
    'receive_options','Do not receive data','send_dimensions','off', ...
    'receive_dimensions','off','send_buffer_size','65536', ...
    'receive_buffer_size','1024','send_fifo_size','8', ...
    'implementation','Use blocking I/O in a separate thread', ...
    'optimization','Minimum latency');
% enum 1 in the generated QUARC API is little endian; set the mask explicitly.
set_param(server,'byte_order','little endian (Intel - LSB first)');
add_block('simulink/Sources/Constant',[model '/QGC Enable'], ...
    'Value','1','Position',[1400 1560 1460 1590]);
add_block('simulink/Sources/Constant',[model '/QGC URI'], ...
    'Value','[uint8(''tcpip://localhost:18920'') uint8(0)]', ...
    'Position',[1300 1630 1500 1660]);
% Enforce framing during Update Diagram/code generation, without entering
% normal-simulation compile (which can activate hardware S-functions).
spec=[model '/QGC Packet Width 59'];
add_block('simulink/Signal Attributes/Signal Specification',spec, ...
    'Dimensions','59','Position',[1525 930 1580 960]);
connect(model,'QGC Payload',1,spec,1);
connect(model,'QGC Packet Width 59',1,server,1);
connect(model,'QGC Enable',1,server,2);
connect(model,'QGC URI',1,server,3);
ports=get_param(server,'PortHandles');
for k=1:numel(ports.Outport)
    dest=[model '/QGC Unused ' num2str(k)];
    add_block('simulink/Sinks/Terminator',dest,'Position',[1850 950+40*k 1870 970+40*k]);
    dp=get_param(dest,'PortHandles');add_line(model,ports.Outport(k),dp.Inport(1));
end
for k=1:numel(original)
    assert(get_param(get_param(original(k).dst,'Line'),'SrcPortHandle')==original(k).src, ...
        'An original control connection changed.');
end
% Update checks dimensions/rates/tag visibility before saving the usable model.
set_param(model,'SimulationCommand','update');
if ~exist(fullfile(project,'saved_data',model),'dir'),mkdir(fullfile(project,'saved_data',model));end
if exist(output,'file'),copyfile(output,[output '.' datestr(now,'yyyymmdd_HHMMSS') '.bak']);end
save_system(model,output);
fprintf('PASS: all original connections retained; telemetry width=59, rate=50 Hz.\n');
if buildExecutable
    set_param(model,'DownloadToTarget','off');
    slbuild(model);
    fprintf('PASS: local build. No download, Start or Arm.\n');
end
fprintf('Created %s. TCP telemetry port=18920.\n',output);
end

function connect(sys,name,port,destination,destPort)
sp=get_param([sys '/' strrep(name,'/','//')],'PortHandles');
dp=get_param(destination,'PortHandles');
add_line(sys,sp.Outport(port),dp.Inport(destPort),'autorouting','on');
end

function result=edges(model)
result=struct('src',{},'dst',{});
blocks=find_system(model,'FollowLinks','off','LookUnderMasks','none','Type','Block');
for k=1:numel(blocks)
    ports=get_param(blocks{k},'PortHandles');
    for dst=ports.Inport(:)'
        line=get_param(dst,'Line');
        if line~=-1
            result(end+1)=struct('src',get_param(line,'SrcPortHandle'),'dst',dst); %#ok<AGROW>
        end
    end
end
end
