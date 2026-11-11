function install_baro_height
folder=fileparts(mfilename('fullpath'));project=fileparts(folder);addpath(folder);
source='Read_IMU';model='Read_IMU_Height';output=fullfile(project,[model '.slx']);
if bdIsLoaded(model),error('Close Read_IMU_Height before installing.');end
if exist(output,'file'),copyfile(output,[output '.' datestr(now,'yyyymmdd_HHMMSS') '.bak']);end
load_system('D:\QDrone\Read_IMU.slx');save_system(source,output);load_system(output);
set_param(model,'InitFcn',sprintf('addpath(fullfile(fileparts(get_param(''%s'',''FileName'')),''capture_tool''));',model));
% Source Barometer/1 is RAW Pa before the example's Bias Removal block.
assert(getSimulinkBlockHandle([model '/Barometer'])~=-1);
blocks=find_system(model,'SearchDepth',1,'Type','Block');imu='';
for k=1:numel(blocks)
 p=get_param(blocks{k},'DialogParameters');if ~isstruct(p),continue;end
 if isfield(p,'analog_channels')&&isfield(p,'other_channels')&&~strcmp(get_param(blocks{k},'analog_channels'),'[]'),imu=blocks{k};end
 if isfield(p,'board_type')&&strcmp(get_param(blocks{k},'board_type'),'qdrone')
  keys=fieldnames(p);for j=1:numel(keys)
   if ~isempty(regexp(keys{j},'^set_.*_(at_start|at_switch_in|at_terminate|at_switch_out|on_watchdog)$','once')),set_param(blocks{k},keys{j},'off');end
  end
  for key={'analog_output_channels','digital_output_channels','pwm_channels','other_output_channels','encoder_channels'}
   if isfield(p,key{1}),set_param(blocks{k},key{1},'[]');end
  end
 end
end
assert(~isempty(imu),'Raw IMU source missing');
sub=[model '/Ground Reference Height'];add_block('built-in/Subsystem',sub,'Position',[800 800 1100 970]);
add_block('built-in/Inport',[sub '/Raw pressure Pa'],'Port','1','Position',[20 30 50 50]);
add_block('built-in/Inport',[sub '/Raw gyro accel'],'Port','2','Position',[20 100 50 120]);
add_block('simulink/Signal Attributes/Rate Transition',[sub '/Pressure Rate'],'OutPortSampleTime','0.025','Position',[90 25 140 60]);
add_block('simulink/Signal Attributes/Rate Transition',[sub '/IMU Rate'],'OutPortSampleTime','0.025','Position',[90 90 140 125]);
add_block('simulink/Sources/Digital Clock',[sub '/Time'],'sampleTime','0.025','Position',[90 150 140 175]);
add_block('simulink/Sources/Constant',[sub '/Rezero toggle 0 to 1'],'Value','0','SampleTime','0.025','Position',[20 220 100 250]);
add_block('simulink/Logic and Bit Operations/Detect Rise Positive',[sub '/Reset pulse'],'Position',[130 215 180 255]);
chartPath=[sub '/Barometric Height'];add_block('simulink/User-Defined Functions/MATLAB Function',chartPath,'Position',[230 40 450 200]);
sf=sfroot;chart=sf.find('-isa','Stateflow.EMChart','Path',chartPath);
chart.Script=sprintf(['function y=run(p,imu,t,reset)\n%%#codegen\n' ...
 'g=sqrt(sum(imu(1:3).^2));a=sqrt(sum(imu(4:6).^2));\n' ...
 'y=qdrone_baro_height(p,t,g,a,reset);\nend\n']);
add_line(sub,'Raw pressure Pa/1','Pressure Rate/1');add_line(sub,'Pressure Rate/1','Barometric Height/1');
add_line(sub,'Raw gyro accel/1','IMU Rate/1');add_line(sub,'IMU Rate/1','Barometric Height/2');add_line(sub,'Time/1','Barometric Height/3');
add_line(sub,'Rezero toggle 0 to 1/1','Reset pulse/1');add_line(sub,'Reset pulse/1','Barometric Height/4');
labels={'Relative Height m','Raw Height m','Vertical Speed m_s','Height Valid','Ground Pressure Pa','Calibration Progress','Calibration Noise m','Status Code'};
for k=1:8
 selector=[sub '/Select ' num2str(k)];display=[sub '/' labels{k}];
 add_block('simulink/Signal Routing/Selector',selector,'InputPortWidth','8','Indices',num2str(k),'Position',[510 25+65*k 550 50+65*k]);
 add_block('simulink/Sinks/Display',display,'Position',[600 20+65*k 770 55+65*k]);
 add_line(sub,'Barometric Height/1',['Select ' num2str(k) '/1'],'autorouting','on');add_line(sub,['Select ' num2str(k) '/1'],[labels{k} '/1']);
end
add_block('simulink/Sinks/Scope',[sub '/Height History'],'Position',[830 80 900 130]);
add_line(sub,'Select 1/1','Height History/1','autorouting','on');
p=get_param(sub,'PortHandles');b=get_param([model '/Barometer'],'PortHandles');i=get_param(imu,'PortHandles');
add_line(model,b.Outport(1),p.Inport(1),'autorouting','on');add_line(model,i.Outport(2),p.Inport(2),'autorouting','on');
note=Simulink.Annotation(sub,sprintf(['BAROMETRIC RELATIVE HEIGHT - NOT RANGE TO FLOOR\n' ...
 'Place on ground and keep still for 20 seconds to establish zero.\n' ...
 'Rezero: change constant 0 -> 1. Use 1 -> 0 -> 1 for another reset.\n' ...
 'Height Valid must be 1. NaN means unavailable. Weather/airflow cause drift.']));note.Position=[20 330];
set_param(model,'SimulationCommand','update');save_system(model);
fprintf('PASS: saved %s. No target downloaded or started.\n',output);
close_system(model,0);
end


