function build_qdrone_capture(includeRealSense,droneIP,recordPeriod)
% Create a sensor-only model. Does not build, download, start or arm hardware.
if nargin<1,includeRealSense=true;end
if nargin<2,droneIP='192.168.2.52';end
if nargin<3,recordPeriod=0.099;end
assert(recordPeriod>=0.033 && abs(recordPeriod/0.033-round(recordPeriod/0.033))<1e-8, ...
    'recordPeriod must be an integer multiple of 0.033 seconds.');
folder=fileparts(mfilename('fullpath'));project=fileparts(folder);
addpath(folder,fullfile(project,'optical_flow'));
model='QDrone_Capture';output=fullfile(project,[model '.slx']);
if bdIsLoaded(model),error('Close QDrone_Capture before rebuilding its diagram.');end
if exist(output,'file'),copyfile(output,[output '.' datestr(now,'yyyymmdd_HHMMSS') '.bak']);end
load_system('D:\QDrone\Read_IMU.slx');load_system('D:\QDrone\Read_Cameras.slx');
new_system(model);cfg=copy(getActiveConfigSet('Read_IMU'));cfg.Name='CaptureConfig';
attachConfigSet(model,cfg);setActiveConfigSet(model,cfg.Name);
set_param(model,'FixedStep','0.001','StopTime','inf','SimulationMode','external');
set_param(model,'InitFcn',sprintf(['p=fileparts(get_param(''%s'',''FileName''));' ...
    'addpath(fullfile(p,''capture_tool''),fullfile(p,''optical_flow''));'],model));
set_param(model,'ExtModeMexArgs',sprintf('''-w -d /tmp -uri %%u'',''tcpip://%s:17005''',droneIP));
hil=[model '/Sensors Initialize'];add_block('Read_IMU/HIL Initialize',hil,'Position',[20 20 160 90]);
params=get_param(hil,'DialogParameters');keys=fieldnames(params);
for k=1:numel(keys)
    if ~isempty(regexp(keys{k},'^set_.*_(at_start|at_switch_in|at_terminate|at_switch_out|on_watchdog)$','once'))
        set_param(hil,keys{k},'off');
    end
end
for item={'analog_output_channels','digital_output_channels','pwm_channels','other_output_channels','encoder_channels'}
    if isfield(params,item{1}),set_param(hil,item{1},'[]');end
end
set_param(hil,'object_name','CaptureSensors','exclusive','on');
refs=find_system('Read_IMU','SearchDepth',1,'Type','Block');template='';
for k=1:numel(refs)
    d=get_param(refs{k},'DialogParameters'); if isstruct(d) && isfield(d,'analog_channels') && isfield(d,'other_channels') && ~strcmp(get_param(refs{k},'analog_channels'),'[]')
        template=refs{k};break;
    end
end
assert(~isempty(template),'HIL Read template missing');
read=[model '/All Sensor Inputs'];add_block(template,read,'Position',[20 130 160 220]);
set_param(read,'object_name','CaptureSensors','analog_channels','0:4', ...
    'other_channels','[3000:3002 4000:4002]', ...
    'sample_time','0.001','vector_output','on');
add_block('simulink/Sources/Digital Clock',[model '/Sensor Time'],'sampleTime','0.001','Position',[20 270 100 300]);
add_block('simulink/Sources/Digital Clock',[model '/Frame Time'],'sampleTime','0.033','Position',[20 530 100 560]);
camera=[model '/Down Camera'];add_block('Read_Cameras/Video Capture',camera,'Position',[20 360 160 440]);
set_param(camera,'sample_time','0.033');
flow=[model '/Flow'];add_block('simulink/User-Defined Functions/MATLAB Function',flow,'Position',[240 360 420 470]);
sf=sfroot;chart=sf.find('-isa','Stateflow.EMChart','Path',flow);
chart.Script=sprintf(['function [flow,sequence,frameTime]=run(image,fresh,t)\n%%#codegen\n' ...
    '[flow,sequence,frameTime]=qdrone_capture_flow(image,fresh,t);\nend\n']);
add_line(model,'Down Camera/1','Flow/1');add_line(model,'Down Camera/2','Flow/2');add_line(model,'Frame Time/1','Flow/3');
for k=1:3
    rate=[model '/Frame To Sensor ' num2str(k)];
    add_block('simulink/Signal Attributes/Rate Transition',rate,'OutPortSampleTime','0.001', ...
        'Position',[460 350+55*k 520 380+55*k]);
    add_line(model,['Flow/' num2str(k)],['Frame To Sensor ' num2str(k) '/1'],'autorouting','on');
end
packet=[model '/Telemetry Packet'];add_block('simulink/User-Defined Functions/MATLAB Function',packet,'Position',[610 130 810 310]);
chart=sf.find('-isa','Stateflow.EMChart','Path',packet);
chart.Script=sprintf(['function p=run(t,a,s,f,seq,stamp)\n%%#codegen\n' ...
    'p=qdrone_capture_packet(t,a,s,f,seq,stamp);\nend\n']);
add_line(model,'Sensor Time/1','Telemetry Packet/1');
add_line(model,'All Sensor Inputs/1','Telemetry Packet/2');add_line(model,'All Sensor Inputs/2','Telemetry Packet/3');
for k=1:3,add_line(model,['Frame To Sensor ' num2str(k) '/1'],['Telemetry Packet/' num2str(k+3)]);end
server(model,'Telemetry',18910,0.001,30*8,'Telemetry Packet/1',20000);
% Down camera is processed at 30.3 Hz, images recorded at configurable lower rate.
frameServer(model,'Down',1,'Down Camera/1','Down Camera/2','Frame Time/1',recordPeriod,[20 640 170 710]);
if includeRealSense
    add_block('Read_Cameras/Video3D Initialize',[model '/RealSense Initialize'],'Position',[20 800 170 870]);
    set_param([model '/RealSense Initialize'],'sample_time','0.033');
    rsNames={'RGB Data','Depth Data','Left Infrared Data','Right Infrared Data'};
    labels={'RGB','Depth','LeftIR','RightIR'};
    for k=1:4
        b=[model '/' labels{k} ' Camera'];
        add_block(['Read_Cameras/' rsNames{k}],b,'Position',[230 780+160*(k-1) 370 860+160*(k-1)]);
        set_param(b,'sample_time','0.033');
        clock=[model '/' labels{k} ' Time'];add_block('simulink/Sources/Digital Clock',clock, ...
            'sampleTime','0.033','Position',[230 875+160*(k-1) 310 905+160*(k-1)]);
        frameServer(model,labels{k},k+1,[labels{k} ' Camera/1'],[labels{k} ' Camera/4'],[labels{k} ' Time/1'],0.495,[430 780+160*(k-1) 580 860+160*(k-1)]);
        % Unused metadata outputs are intentionally terminated below.
    end
end
% Terminate every unused output, including network diagnostics.
blocks=find_system(model,'SearchDepth',1,'Type','Block');counter=0;
for k=1:numel(blocks)
    if strcmp(blocks{k},model),continue;end
    ports=get_param(blocks{k},'PortHandles');
    for j=1:numel(ports.Outport)
        if get_param(ports.Outport(j),'Line')==-1
            counter=counter+1;term=[model '/Unused ' num2str(counter)];
            add_block('simulink/Sinks/Terminator',term,'Position',[1200 30+25*counter 1220 50+25*counter]);
            tp=get_param(term,'PortHandles');add_line(model,ports.Outport(j),tp.Inport(1),'autorouting','on');
        end
    end
end
assert(isempty(find_system(model,'RegExp','on','Name','(?i).*HIL.*Write.*')),'Unexpected HIL Write');
annotation=Simulink.Annotation(model,sprintf(['SENSOR COLLECTION ONLY - NO MOTOR OUTPUTS\n' ...
    'Stop all other QDrone targets first. Remove propellers.\n' ...
    'Telemetry: port 18910, 1 kHz. Down image processing: 30.3 Hz.\n' ...
    'RealSense: 2.02 Hz when enabled. Timestamp = software read time.']));annotation.Position=[600 600];
set_param(model,'SimulationCommand','update');save_system(model,output);
manifest=struct('schema',1,'model',model,'drone_ip',droneIP,'imu_period_s',0.001, ...
    'flow_period_s',0.033,'down_record_period_s',recordPeriod,'realsense_capture_period_s',0.033,'realsense_record_period_s',0.495, ...
    'include_realsense',logical(includeRealSense),'motor_outputs_enabled',false, ...
    'range_sensor_available',false,'camera_timestamp','target software read time; not hardware exposure time', ...
    'not_collected',{{'range','motor RPM/feedback','FrSky commands','hardware exposure timestamps'}});
file=fopen(fullfile(folder,'capture_schema.json'),'w');assert(file>=0);fprintf(file,'%s',jsonencode(manifest));fclose(file);
fprintf('CREATED %s. Diagram update passed. No target build/start performed.\n',output);
end
function server(model,label,port,period,bytes,source,fifo)
path=[model '/' label ' Server'];
add_block('quarc_library/Communications/Basic/Stream Server',path,'Position',[920 40+port-18910 1070 150+port-18910]);
set_param(path,'uri_source','External input port','uri',sprintf('tcpip://localhost:%d',port), ...
    'sample_time',num2str(period,15),'active','on','send_options','Send all data', ...
    'receive_options','Do not receive data','send_dimensions','off','receive_dimensions','off', ...
    'send_buffer_size',num2str(max(65536,bytes*2)), ...
    'receive_buffer_size','1024','send_fifo_size',num2str(fifo), ...
    'implementation','Use blocking I/O in a separate thread','optimization','Minimum latency');
add_block('simulink/Sources/Constant',[model '/' label ' Enable'],'Value','1','Position',[780 60+port-18910 810 80+port-18910]);
add_block('simulink/Sources/Constant',[model '/' label ' URI'],'Value', ...
    sprintf('[uint8(''tcpip://localhost:%d'') uint8(0)]',port),'Position',[780 95+port-18910 900 115+port-18910]);
add_line(model,source,[label ' Server/1'],'autorouting','on');
add_line(model,[label ' Enable/1'],[label ' Server/2'],'autorouting','on');
add_line(model,[label ' URI/1'],[label ' Server/3'],'autorouting','on');
end
function frameServer(model,label,id,image,newFlag,time,period,position)
p=[model '/' label ' Frame Packet'];
add_block('simulink/User-Defined Functions/MATLAB Function',p,'Position',position);
sf=sfroot;chart=sf.find('-isa','Stateflow.EMChart','Path',p);
chart.Script=sprintf(['function packet=run(image,fresh,t)\n%%#codegen\n' ...
    'packet=qdrone_capture_frame(image,fresh,t,%d);\nend\n'],id);
imageData=chart.find('-isa','Stateflow.Data','Name','image');
if id==2,imageData.Props.Array.Size='[480 640 3]';else,imageData.Props.Array.Size='[480 640]';end
if id==3,imageData.DataType='uint16';else,imageData.DataType='uint8';end
freshData=chart.find('-isa','Stateflow.Data','Name','fresh');freshData.DataType='boolean';
outputData=chart.find('-isa','Stateflow.Data','Name','packet');outputData.DataType='uint8';
bytes=640*480;if id==2,bytes=bytes*3;elseif id==3,bytes=bytes*2;end
outputData.Props.Array.Size=sprintf('[%d 1]',bytes+60);
add_line(model,image,[label ' Frame Packet/1'],'autorouting','on');
add_line(model,newFlag,[label ' Frame Packet/2'],'autorouting','on');
add_line(model,time,[label ' Frame Packet/3'],'autorouting','on');
rate=[model '/' label ' Record Rate'];
add_block('simulink/Signal Attributes/Rate Transition',rate,'OutPortSampleTime',num2str(period,15), ...
    'Position',position+[200 0 130 -20]);
add_line(model,[label ' Frame Packet/1'],[label ' Record Rate/1']);
bytes=640*480;if id==2,bytes=bytes*3;elseif id==3,bytes=bytes*2;end
server(model,label,18910+id,period,bytes+60,[label ' Record Rate/1'],8);
end




