function setup_roll_integral_trial(buildExecutable)
% Prepare a bounded roll-I trial on a separate AutoCal copy. Never Start/Arm.
if nargin==0, buildExecutable=false; end
folder=fileparts(mfilename('fullpath')); project=fileparts(folder);
cd(project);addpath(folder);test_roll_i;
source='Stabilizer_QDrone_048378_AutoCal';model='Stabilizer_QDrone_048378_RollI';
assert(~bdIsLoaded(model),'Close RollI before setup.');
output=fullfile(project,[model '.mdl']);
if exist(output,'file'),copyfile(output,[output '.' datestr(now,'yyyymmdd_HHMMSS') '.bak']);end
f=fopen(fullfile(project,[source '.mdl']),'r','n','windows-1252');assert(f~=-1,'AutoCal model missing.');
body=fread(f,Inf,'*char')';fclose(f);body=strrep(body,source,model);
f=fopen(output,'w','n','windows-1252');assert(f~=-1);fwrite(f,body,'char');fclose(f);
load_system(output);cleanup=onCleanup(@()close_system(model,0));
p=[model '/PHYSICAL PLANT w// LINEAR THRUST'];s=[model '/STABILIZER - QDRONE'];

% Route Ready and pre-limit motor commands across the two subsystems.
add_block('simulink/Signal Routing/Goto',[p '/RollI Ready Goto'], ...
    'GotoTag','RollI_CalibrationReady','TagVisibility','global','Position',[2080 1700 2200 1720]);
add_block('simulink/Signal Routing/Goto',[p '/RollI Motors Goto'], ...
    'GotoTag','RollI_RawMotors','TagVisibility','global','Position',[2080 1740 2200 1760]);
connect(p,'Automatic Gyro Calibration',3,'RollI Ready Goto',1);
connect(p,'Product',1,'RollI Motors Goto',1);
add_block('simulink/Signal Routing/From',[s '/RollI Ready'],'GotoTag','RollI_CalibrationReady','Position',[1320 1510 1430 1530]);
add_block('simulink/Signal Routing/From',[s '/RollI Raw Motors'],'GotoTag','RollI_RawMotors','Position',[1320 1550 1430 1570]);

% Detect clipping of the allocator or torque limiter; delay by one base tick.
limit=[s '/RollI Limit Detection'];
add_block('simulink/User-Defined Functions/MATLAB Function',limit,'Position',[1470 1540 1640 1620]);
make_chart(limit,sprintf(['function y=detect(motor,rawTorque,limitedTorque)\n%%#codegen\n' ...
    'y=double(any(~isfinite(motor(:))) || any(motor(:)<0) || any(motor(:)>1) || ' ...
    'any(~isfinite(rawTorque(:))) || any(~isfinite(limitedTorque(:))) || ' ...
    'any(abs(rawTorque(:)-limitedTorque(:))>1e-8));\nend\n']), ...
    {'motor','rawTorque','limitedTorque'},{'[4 1]','[3 1]','[3 1]'});
add_block('simulink/Discrete/Unit Delay',[s '/RollI Previous Limit'], ...
    'SampleTime','0.001','InitialCondition','1','Position',[1690 1550 1730 1580]);
column_input(s,'RollI Raw Motors',1,'RollI Limit Detection',1,4,'RollI Limit Motor Column');
column_input(s,'Sum4',1,'RollI Limit Detection',2,3,'RollI Limit Raw Torque Column');
column_input(s,sprintf('Saturation\nCommand \nAuthority (Nm)'),1, ...
    'RollI Limit Detection',3,3,'RollI Limit Saturated Torque Column');
connect(s,'RollI Limit Detection',1,'RollI Previous Limit',1);

cal=[s '/Bounded Roll Integral'];
add_block('simulink/User-Defined Functions/MATLAB Function',cal,'Position',[1800 1470 2000 1650]);
make_chart(cal,sprintf(['function [term,active]=step(errorRate,arm,throttle,ready,mode,angles,gyro,limited)\n%%#codegen\n' ...
    '[term,active]=qdrone_roll_i_step(errorRate,arm,throttle,ready,mode,angles,gyro,limited);\nend\n']), ...
    {'errorRate','angles','gyro','term'},{'[3 1]','[3 1]','[3 1]','[3 1]'});
add_block('simulink/Math Operations/Reshape',[s '/RollI Torque 1D'], ...
    'OutputDimensionality','1-D array','Position',[2030 1470 2080 1500]);
add_block('simulink/Math Operations/Sum',[s '/PD Plus Roll I'], ...
    'Inputs','++','Position',[2120 1470 2150 1530]);
add_block('simulink/Sinks/Display',[s '/RollI Active'],'Position',[2030 1580 2150 1610]);
sources={'Sum2','Arm (0,1)','Throttle (N)','RollI Ready','Mode (0,1)', ...
    'Select Angular Position',sprintf('Select Angular\nSpeed'),'RollI Previous Limit'};
for k=1:8
    if any(k==[1 6 7])
        column_input(s,sources{k},1,'Bounded Roll Integral',k,3,sprintf('RollI Input %d Column',k));
    else
        connect(s,sources{k},1,'Bounded Roll Integral',k);
    end
end
connect(s,'Bounded Roll Integral',1,'RollI Torque 1D',1);
connect(s,'Bounded Roll Integral',2,'RollI Active',1);
% Preserve every original PD torque destination, including diagnostic logging.
pd=ports([s '/Sum3']);total=ports([s '/PD Plus Roll I']);
dest=move_sinks(s,pd.Outport(1),total.Outport(1));
connect(s,'Sum3',1,'PD Plus Roll I',1);
connect(s,'RollI Torque 1D',1,'PD Plus Roll I',2);

% Log time, integral term, enable, rate error, Ready, prior clip, original PD.
add_block('simulink/Signal Routing/Mux',[s '/RollI Telemetry'],'Inputs','7','Position',[2210 1460 2215 1710]);
add_block([s '/PID Diagnostic Log'],[s '/RollI Log'],'Position',[2280 1510 2420 1570]);
set_param([s '/RollI Log'],'file_name','saved_data/%m/RollI_%{char(datetime)}.mat', ...
    'variable_name','roll_i_data','decimation','2');
logs={'PID Diagnostic Time','RollI Torque 1D','Bounded Roll Integral','Sum2','RollI Ready','RollI Previous Limit','Sum3'};
logPorts=[1 1 2 1 1 1 1];
for k=1:7,connect(s,logs{k},logPorts(k),'RollI Telemetry',k);end
connect(s,'RollI Telemetry',1,'RollI Log',1);
assert(strcmp(get_param(model,'FixedStep'),'0.001'));
set_param(model,'SimulationCommand','update');
for h=dest(:)',assert_source(h,total.Outport(1));end
pp=ports([s '/PD Plus Roll I']);ip=ports([s '/RollI Torque 1D']);
assert_source(pp.Inport(1),pd.Outport(1));assert_source(pp.Inport(2),ip.Outport(1));
% Existing P/D gains must match the saved AutoCal copy exactly.
referenceLoaded=bdIsLoaded(source);load_system(fullfile(project,[source '.mdl']));
for name={'K_p_angle (rad/s / rad)','K_d_angle (rad/s / rad/s)', ...
        'K_p_speed (Nm / rad/s)','K_d_speed (Nm / rad/s/s)'}
    escaped=strrep(name{1},'/','//');
    assert(strcmp(get_param([s '/' escaped],'Gain'), ...
        get_param([source '/STABILIZER - QDRONE/' escaped],'Gain')),'Original P/D gain changed.');
end
if ~referenceLoaded,close_system(source,0);end
mkdir(fullfile(project,'saved_data',model));save_system(model,output);
fprintf('PASS: RollI diagram, preserved PD gains, torque routing and telemetry.\n');
if buildExecutable
    set_param(model,'DownloadToTarget','off');slbuild(model);
    fprintf('PASS: RollI local build. No download, Start or Arm.\n');
end
fprintf('Created %s\n',output);clear cleanup
end
function make_chart(path,script,names,sizes)
r=sfroot;c=r.find('-isa','Stateflow.EMChart','Path',path);assert(numel(c)==1);
c.Script=script;c.ChartUpdate='DISCRETE';c.SampleTime='0.001';
for k=1:numel(names),d=c.find('-isa','Stateflow.Data','Name',names{k});assert(numel(d)==1);d.Props.Array.Size=sizes{k};end
end
function h=ports(path),h=get_param(path,'PortHandles');end
function column_input(system,source,out,destination,in,width,name)
% Isolate a matrix-valued chart port from the original 1-D signal network.
% This prevents chart dimension constraints from changing existing limiter/gain ports.
add_block('simulink/Math Operations/Reshape',[system '/' name], ...
    'OutputDimensionality','Customize','OutputDimensions',sprintf('[%d 1]',width), ...
    'Position',[1460 1800+in*50 1540 1830+in*50]);
connect(system,source,out,name,1);
connect(system,name,1,destination,in);
a=ports([system '/' source]);b=ports([system '/' name]);d=ports([system '/' destination]);
assert_source(b.Inport(1),a.Outport(out));
assert_source(d.Inport(in),b.Outport(1));
end
function connect(system,source,out,destination,in)
a=ports([system '/' source]);b=ports([system '/' destination]);
add_line(system,a.Outport(out),b.Inport(in),'autorouting','on');
end
function d=move_sinks(system,source,replacement)
l=get_param(source,'Line');assert(l~=-1);d=get_param(l,'DstPortHandle');assert(~isempty(d)&&all(d~=-1));
delete_line(l);for k=1:numel(d),add_line(system,replacement,d(k),'autorouting','on');end
end
function assert_source(input,expected)
l=get_param(input,'Line');assert(l~=-1&&get_param(l,'SrcPortHandle')==expected,'Incorrect roll I routing.');
end
