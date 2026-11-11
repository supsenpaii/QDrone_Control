function build_capture_from_read_imu
folder=fileparts(mfilename('fullpath'));project=fileparts(folder);addpath(folder);
source='Read_IMU';model='QDrone_Capture_IMU';output=fullfile(project,[model '.slx']);
if bdIsLoaded(model),error('Close QDrone_Capture_IMU first.');end
load_system('D:\QDrone\Read_IMU.slx');
if exist(output,'file'),copyfile(output,[output '.' datestr(now,'yyyymmdd_HHMMSS') '.bak']);end
save_system(source,output);load_system(output);
% Match all three URI fields used by the working Read_IMU model.
set_param(model,'ExtModeMexArgs','''-w -d /tmp -uri %u'', ''tcpip://192.168.2.52:17001?retries=10'', ''tcpip://192.168.2.52:17000''');
set_param(model,'InitFcn',sprintf('addpath(fullfile(fileparts(get_param(''%s'',''FileName'')),''capture_tool''));',model));
blocks=find_system(model,'SearchDepth',1,'Type','Block');imu='';mag='';baro='';
for k=1:numel(blocks)
    p=get_param(blocks{k},'DialogParameters');if ~isstruct(p),continue;end
    if isfield(p,'analog_channels') && isfield(p,'other_channels')
        channels=get_param(blocks{k},'other_channels');
        if ~strcmp(get_param(blocks{k},'analog_channels'),'[]'),imu=blocks{k};
        elseif ~isempty(strfind(channels,'8000')),mag=blocks{k};
        elseif ~isempty(strfind(channels,'9000')),baro=blocks{k};end
    end
    % Disable output configuration/writes inherited from the example.
    if isfield(p,'board_type') && strcmp(get_param(blocks{k},'board_type'),'qdrone')
        keys=fieldnames(p);
        for j=1:numel(keys)
            if ~isempty(regexp(keys{j},'^set_.*_(at_start|at_switch_in|at_terminate|at_switch_out|on_watchdog)$','once'))
                set_param(blocks{k},keys{j},'off');
            end
        end
        for key={'analog_output_channels','digital_output_channels','pwm_channels','other_output_channels','encoder_channels'}
            if isfield(p,key{1}),set_param(blocks{k},key{1},'[]');end
        end
    end
end
assert(~isempty(imu)&&~isempty(mag)&&~isempty(baro),'Cannot identify original raw sensor blocks');
add_block('simulink/Sources/Digital Clock',[model '/Capture Clock'],'sampleTime','0.001','Position',[1100 50 1180 80]);
packet=[model '/Capture Telemetry'];add_block('simulink/User-Defined Functions/MATLAB Function',packet,'Position',[1400 80 1610 260]);
sf=sfroot;chart=sf.find('-isa','Stateflow.EMChart','Path',packet);
chart.Script=sprintf(['function p=run(t,battery,imu,mag,baro)\n%%#codegen\n' ...
    'p=[51234567;1;round(t/0.001);t;nan(4,1);battery;imu(:);mag(:);baro;NaN;zeros(8,1);0;0];\nend\n']);
add_line(model,'Capture Clock/1','Capture Telemetry/1');
origins={imu,imu,mag,baro};outIndex=[1 2 1 1];
for k=1:4
    name=['Capture Rate ' num2str(k)];rate=[model '/' name];
    add_block('simulink/Signal Attributes/Rate Transition',rate,'OutPortSampleTime','0.001','Position',[1200 100+70*k 1260 140+70*k]);
    a=get_param(origins{k},'PortHandles');b=get_param(rate,'PortHandles');c=get_param(packet,'PortHandles');
    add_line(model,a.Outport(outIndex(k)),b.Inport(1),'autorouting','on');
    add_line(model,b.Outport(1),c.Inport(k+1),'autorouting','on');
end
server=[model '/Capture Server'];add_block('quarc_library/Communications/Basic/Stream Server',server,'Position',[1800 100 1950 240]);
set_param(server,'uri_source','External input port','sample_time','0.001','active','on', ...
    'send_options','Send all data','receive_options','Do not receive data', ...
    'send_dimensions','off','receive_dimensions','off','send_buffer_size','65536', ...
    'receive_buffer_size','1024','send_fifo_size','20000', ...
    'implementation','Use blocking I/O in a separate thread','optimization','Minimum latency');
add_block('simulink/Sources/Constant',[model '/Capture Enable'],'Value','1','Position',[1650 280 1690 310]);
add_block('simulink/Sources/Constant',[model '/Capture URI'], ...
    'Value','[uint8(''tcpip://localhost:18910'') uint8(0)]','Position',[1650 340 1790 375]);
add_line(model,'Capture Telemetry/1','Capture Server/1');
add_line(model,'Capture Enable/1','Capture Server/2');add_line(model,'Capture URI/1','Capture Server/3');
ports=get_param(server,'PortHandles');for k=1:numel(ports.Outport)
    name=['Capture Unused ' num2str(k)];add_block('simulink/Sinks/Terminator',[model '/' name], 'Position',[2010 80+50*k 2030 100+50*k]);
    p=get_param([model '/' name],'PortHandles');add_line(model,ports.Outport(k),p.Inport(1));
end
note=Simulink.Annotation(model,sprintf(['IMU CAPTURE BASELINE FROM WORKING Read_IMU\n' ...
    'No motor outputs. Raw gyro/accel/magnetometer/barometer/battery.\n' ...
    'No camera or flow yet. Host: start_qdrone_capture(''192.168.2.52'',false).']));note.Position=[1100 440];
set_param(model,'SimulationCommand','update');save_system(model);
fprintf('PASS: saved %s; diagram update passed. No target downloaded or started.\n',output);
close_system(model,0);
end
