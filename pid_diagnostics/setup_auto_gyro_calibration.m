function setup_auto_gyro_calibration(buildExecutable)
% Create and validate a per-run gyro calibration copy. Never Start or Arm.
if nargin==0, buildExecutable=false; end
folder=fileparts(mfilename('fullpath')); project=fileparts(folder);
cd(project); addpath(folder);
test_gyrocal;
source='Stabilizer_QDrone_048378_PID_Diag';
model='Stabilizer_QDrone_048378_AutoCal';
assert(~bdIsLoaded(model),'Close AutoCal before setup.');
output=fullfile(project,[model '.mdl']);
if exist(output,'file'), copyfile(output,[output '.' datestr(now,'yyyymmdd_HHMMSS') '.bak']); end
% Copy the saved passive model, not the failed fixed-bias experiment.
f=fopen(fullfile(project,[source '.mdl']),'r','n','windows-1252');
assert(f~=-1,'Cannot read diagnostic model.');
body=fread(f,Inf,'*char')'; fclose(f);
body=strrep(body,source,model);
f=fopen(output,'w','n','windows-1252'); assert(f~=-1,'Cannot write AutoCal copy.');
fwrite(f,body,'char'); fclose(f);
load_system(output);
cleanup=onCleanup(@() close_system(model,0));
plant=[model '/PHYSICAL PLANT w// LINEAR THRUST'];
cal=[plant '/Automatic Gyro Calibration'];
add_block('simulink/User-Defined Functions/MATLAB Function',cal,'Position',[1330 1510 1530 1610]);
root=sfroot;
chart=root.find('-isa','Stateflow.EMChart','Path',cal);
assert(numel(chart)==1,'Cannot find calibration chart.');
chart.Script=sprintf(['function [gyro,bias,ready,progress]=calibrate(raw,accel,arm)\n' ...
    '%%#codegen\n[gyro,bias,ready,progress]=qdrone_gyrocal_step(raw,accel,arm);\nend\n']);
chart.ChartUpdate='DISCRETE';
chart.SampleTime='0.001';
% Define vector sizes explicitly for diagram compilation and code generation.
for name={'raw','accel','gyro','bias'}
    data=chart.find('-isa','Stateflow.Data','Name',name{1});
    assert(numel(data)==1,'Missing chart data %s.',name{1});
    data.Props.Array.Size='[3 1]';
end
% The original controller uses 1-D vectors, not 3-by-1 matrix signals.
% Keep the chart's column arithmetic internal and normalize both vector outputs.
gyroVector=[plant '/Calibrated Gyro 1D'];
biasVector=[plant '/Estimated Bias 1D'];
add_block('simulink/Math Operations/Reshape',gyroVector, ...
    'OutputDimensionality','1-D array','Position',[1550 1470 1600 1500]);
add_block('simulink/Math Operations/Reshape',biasVector, ...
    'OutputDimensionality','1-D array','Position',[1550 1510 1600 1530]);
gate=[plant '/Arm After Gyro Calibration'];
add_block('simulink/Math Operations/Product',gate,'Inputs','2','Position',[1330 1640 1390 1680]);
readyDisplay=[plant '/Calibration Ready 0 or 1'];
add_block('simulink/Sinks/Display',readyDisplay,'Position',[1600 1540 1750 1570]);
add_block('simulink/Sinks/Display',[plant '/Calibration Progress 0 to 1'],'Position',[1600 1590 1750 1620]);

% Replace only the gyro source and the physical arm source; preserve every sink.
demux=get_param([plant '/Demux'],'PortHandles');
calPorts=get_param(cal,'PortHandles');
arm=get_param([plant '/Arm (0,1)'],'PortHandles');
gatePorts=get_param(gate,'PortHandles');
gyroVectorPorts=get_param(gyroVector,'PortHandles');
biasVectorPorts=get_param(biasVector,'PortHandles');
gyroDestinations=move_sinks(plant,demux.Outport(1),gyroVectorPorts.Outport(1));
armDestinations=move_sinks(plant,arm.Outport(1),gatePorts.Outport(1));
add_line(plant,calPorts.Outport(1),gyroVectorPorts.Inport(1),'autorouting','on');
add_line(plant,calPorts.Outport(2),biasVectorPorts.Inport(1),'autorouting','on');
add_line(plant,demux.Outport(1),calPorts.Inport(1),'autorouting','on');
add_line(plant,demux.Outport(2),calPorts.Inport(2),'autorouting','on');
add_line(plant,arm.Outport(1),calPorts.Inport(3),'autorouting','on');
add_line(plant,arm.Outport(1),gatePorts.Inport(1),'autorouting','on');
add_line(plant,calPorts.Outport(3),gatePorts.Inport(2),'autorouting','on');
add_line(plant,'Automatic Gyro Calibration/3','Calibration Ready 0 or 1/1','autorouting','on');
add_line(plant,'Automatic Gyro Calibration/4','Calibration Progress 0 to 1/1','autorouting','on');

% Additional raw/compensated/bias/ready telemetry, retaining the existing logs.
mux=[plant '/Calibration Telemetry'];
add_block('simulink/Signal Routing/Mux',mux,'Inputs','8','Position',[1830 1480 1835 1730]);
logger=[plant '/Calibration Log'];
add_block([plant '/PID Diagnostic Log'],logger,'Position',[1900 1550 2020 1610]);
set_param(logger,'file_name','saved_data/%m/GyroCal_%{char(datetime)}.mat', ...
    'variable_name','gyro_cal_data','decimation','2');
inputs=[get_port([plant '/PID Diagnostic Time'],1),demux.Outport(1), ...
    gyroVectorPorts.Outport(1),biasVectorPorts.Outport(1),calPorts.Outport(3),calPorts.Outport(4), ...
    arm.Outport(1),gatePorts.Outport(1)];
mp=get_param(mux,'PortHandles');
for k=1:8, add_line(plant,inputs(k),mp.Inport(k),'autorouting','on'); end
add_line(plant,'Calibration Telemetry/1','Calibration Log/1','autorouting','on');

oldInit=get_param(model,'InitFcn');
set_param(model,'InitFcn',sprintf('addpath(''%s'');\n%s',strrep(folder,'''',''''''),oldInit));
assert(strcmp(get_param(model,'FixedStep'),'0.001'),'Calibration assumes 1 ms base step.');
set_param(model,'SimulationCommand','update');
for h=gyroDestinations(:)'
    assert_source(h,gyroVectorPorts.Outport(1));
end
assert_source(gyroVectorPorts.Inport(1),calPorts.Outport(1));
assert_source(biasVectorPorts.Inport(1),calPorts.Outport(2));
for h=armDestinations(:)'
    assert_source(h,gatePorts.Outport(1));
end
assert_source(calPorts.Inport(1),demux.Outport(1));
assert_source(calPorts.Inport(2),demux.Outport(2));
assert_source(calPorts.Inport(3),arm.Outport(1));
assert_source(gatePorts.Inport(1),arm.Outport(1));
assert_source(gatePorts.Inport(2),calPorts.Outport(3));
% Verify motor arm multiplier actually receives the guarded Arm signal.
product=get_param([plant '/Product1'],'PortHandles');
assert_source(product.Inport(2),gatePorts.Outport(1));
for k=1:8, assert_source(mp.Inport(k),inputs(k)); end
mkdir(fullfile(project,'saved_data',model));
save_system(model,output);
fprintf('PASS: AutoCal diagram, gyro routing, motor Arm gate and calibration telemetry.\n');
if buildExecutable
    set_param(model,'DownloadToTarget','off');
    slbuild(model);
    fprintf('PASS: AutoCal local build. No download, Start or Arm.\n');
end
fprintf('Created %s\n',output);
clear cleanup
end
function destination=move_sinks(system,oldSource,newSource)
line=get_param(oldSource,'Line'); assert(line~=-1,'Missing source line.');
destination=get_param(line,'DstPortHandle');
assert(~isempty(destination) && all(destination~=-1),'Source has an invalid destination.');
delete_line(line);
for k=1:numel(destination), add_line(system,newSource,destination(k),'autorouting','on'); end
end
function port=get_port(block,index)
p=get_param(block,'PortHandles');port=p.Outport(index);
end
function assert_source(input,expected)
line=get_param(input,'Line');
assert(line~=-1 && get_param(line,'SrcPortHandle')==expected,'Missing or incorrect source after rewiring.');
end
