function model=setup_yaw_integral_trial(buildExecutable)
% Separate bounded yaw-I experiment, disabled by default. No download/start/arm.
if nargin<1,buildExecutable=false;end
folder=fileparts(mfilename('fullpath'));project=fileparts(folder);
cd(project);addpath(folder,fullfile(project,'optical_flow'));
test_yaw_i;
source='Stabilizer_QDrone_QGC_OF';model='Stabilizer_QDrone_QGC_OF_YawI';
assert(~bdIsLoaded(model),'Close %s before setup.',model);
assert(~bdIsLoaded(source) || strcmp(get_param(source,'Dirty'),'off'), ...
    'Save the source model first; the saved controller is copied.');
pending=fullfile(project,'qgc_bridge','pending');if ~exist(pending,'dir'),mkdir(pending);end
f=fopen(fullfile(project,[source '.mdl']),'r','n','windows-1252');assert(f~=-1,'Source model missing.');
body=fread(f,Inf,'*char')';fclose(f);body=strrep(body,source,model);
staged=fullfile(pending,[model '.mdl']);
f=fopen(staged,'w','n','windows-1252');assert(f~=-1);fwrite(f,body,'char');fclose(f);
load_system(staged);cleanup=onCleanup(@()close_system(model,0)); %#ok<NASGU>
s=[model '/STABILIZER - QDRONE'];p=[model '/PHYSICAL PLANT w// LINEAR THRUST'];
assert(strcmp(get_param(model,'FixedStep'),'0.001'));
gainNames={'K_p_angle (rad/s / rad)','K_d_angle (rad/s / rad/s)', ...
    'K_p_speed (Nm / rad/s)','K_d_speed (Nm / rad/s/s)'};
oldGains=cell(size(gainNames));
for k=1:numel(gainNames),oldGains{k}=get_param([s '/' strrep(gainNames{k},'/','//')],'Gain');end

% Use the actual source of the plant's motor multiplier Arm input.
mp=ports([p '/Product1']);line=get_param(mp.Inport(2),'Line');assert(line~=-1);
actualArm=get_param(line,'SrcPortHandle');
add_block('simulink/Signal Routing/Goto',[p '/YawI Arm Tap'],'GotoTag','YawI_ActualArm', ...
    'TagVisibility','global','Position',[2300 1900 2420 1920]);
gp=ports([p '/YawI Arm Tap']);add_line(p,actualArm,gp.Inport(1),'autorouting','on');
add_block('simulink/Signal Routing/Goto',[p '/YawI Mixer Tap'],'GotoTag','YawI_RawMotors', ...
    'TagVisibility','global','Position',[2300 1950 2420 1970]);
connect(p,'Product',1,'YawI Mixer Tap',1);
add_block('simulink/Signal Routing/From',[s '/YawI Actual Arm'],'GotoTag','YawI_ActualArm', ...
    'Position',[1400 1900 1540 1920]);
add_block('simulink/Signal Routing/From',[s '/YawI Raw Motors'],'GotoTag','YawI_RawMotors', ...
    'Position',[1400 1950 1540 1970]);

limit=[s '/YawI Limit Detection'];
add_block('simulink/User-Defined Functions/MATLAB Function',limit,'Position',[1650 1950 1850 2050]);
make_chart(limit,sprintf(['function y=detect(motor,rawTorque,limitedTorque)\n%%#codegen\n' ...
    'y=double(any(~isfinite(motor(:))) || any(motor(:)<0) || any(motor(:)>1) || ' ...
    'any(~isfinite(rawTorque(:))) || any(~isfinite(limitedTorque(:))) || ' ...
    'any(abs(rawTorque(:)-limitedTorque(:))>1e-8));\nend\n']), ...
    {'motor','rawTorque','limitedTorque'},{'[4 1]','[3 1]','[3 1]'});
column(s,'YawI Raw Motors',1,'YawI Limit Detection',1,4,'YawI Motor Column');
column(s,'Sum4',1,'YawI Limit Detection',2,3,'YawI Raw Torque Column');
column(s,sprintf('Saturation\nCommand \nAuthority (Nm)'),1,'YawI Limit Detection',3,3,'YawI Limited Torque Column');
add_block('simulink/Discrete/Unit Delay',[s '/YawI Previous Clip'], ...
    'SampleTime','0.001','InitialCondition','1','Position',[1900 1970 1950 2000]);
connect(s,'YawI Limit Detection',1,'YawI Previous Clip',1);

names={'YawI Enable','YawI Ki','YawI Torque Cap','YawI Slew Cap'};
vals={'0','0.01','0.015','0.003'};
for k=1:4
    add_block('simulink/Sources/Constant',[s '/' names{k}],'Value',vals{k}, ...
        'Position',[1400 2050+50*k 1550 2080+50*k]);
end
cal=[s '/Bounded Yaw Integral'];
add_block('simulink/User-Defined Functions/MATLAB Function',cal,'Position',[2050 1880 2300 2250]);
make_chart(cal,sprintf(['function [term,active]=step(errorRate,arm,throttle,mode,angles,gyro,limited,enable,ki,cap,slew)\n%%#codegen\n' ...
    '[term,active]=qdrone_yaw_i_step(errorRate,arm,throttle,mode,angles,gyro,limited,enable,ki,cap,slew);\nend\n']), ...
    {'errorRate','angles','gyro','term'},{'[3 1]','[3 1]','[3 1]','[3 1]'});
sources={'Sum2','YawI Actual Arm','Throttle (N)','Mode (0,1)','Select Angular Position', ...
    sprintf('Select Angular\nSpeed'),'YawI Previous Clip','YawI Enable','YawI Ki','YawI Torque Cap','YawI Slew Cap'};
for k=1:numel(sources)
    if any(k==[1 5 6])
        column(s,sources{k},1,'Bounded Yaw Integral',k,3,sprintf('YawI Input %d Column',k));
    else
        connect(s,sources{k},1,'Bounded Yaw Integral',k);
    end
end
add_block('simulink/Math Operations/Reshape',[s '/YawI Torque 1D'], ...
    'OutputDimensionality','1-D array','Position',[2350 1900 2420 1930]);
connect(s,'Bounded Yaw Integral',1,'YawI Torque 1D',1);
add_block('simulink/Math Operations/Sum',[s '/PD Plus Yaw I'],'Inputs','++', ...
    'Position',[2480 1870 2520 1940]);
% Move all old PD consumers to the summed output, retaining the original PD
% as a separate input/log signal. Torque limiter and motor Arm gate stay in place.
pd=ports([s '/Sum3']);total=ports([s '/PD Plus Yaw I']);
line=get_param(pd.Outport(1),'Line');assert(line~=-1);
dest=get_param(line,'DstPortHandle');assert(~isempty(dest)&&all(dest~=-1));
delete_line(line);
for k=1:numel(dest),add_line(s,total.Outport(1),dest(k),'autorouting','on');end
connect(s,'Sum3',1,'PD Plus Yaw I',1);
connect(s,'YawI Torque 1D',1,'PD Plus Yaw I',2);
add_block('simulink/Sinks/Display',[s '/YawI Active'],'Position',[2360 2080 2500 2110]);
connect(s,'Bounded Yaw Integral',2,'YawI Active',1);

% Dedicated 31-value payload, optionally preceded by the QUARC timestamp.
add_block('simulink/Sources/Digital Clock',[s '/YawI Time'],'sampleTime','0.001', ...
    'Position',[2380 2200 2460 2230]);
add_block('simulink/Signal Routing/Mux',[s '/YawI Telemetry'],'Inputs','17', ...
    'Position',[2650 1900 2655 2450]);
add_block([s '/To Host File'],[s '/YawI Log'],'Position',[2750 2100 2900 2170]);
set_param([s '/YawI Log'],'file_name','saved_data/%m/YawI_%{char(datetime)}.mat', ...
    'variable_name','yaw_i_data','decimation','2','sample_time','0.001');
logs={'YawI Time','Sum2',sprintf('Select Angular\nSpeed'),'Select Angular Position', ...
    'Sum3','YawI Torque 1D','PD Plus Yaw I',sprintf('Saturation\nCommand \nAuthority (Nm)'), ...
    'YawI Actual Arm','Throttle (N)','Mode (0,1)','YawI Enable','Bounded Yaw Integral', ...
    'YawI Previous Clip','YawI Ki','YawI Torque Cap','YawI Slew Cap'};
for k=1:numel(logs)
    out=1;if k==13,out=2;end
    connect(s,logs{k},out,'YawI Telemetry',k);
end
connect(s,'YawI Telemetry',1,'YawI Log',1);
oldInit=get_param(model,'InitFcn');
set_param(model,'InitFcn',sprintf('addpath(''%s'');\n%s',strrep(folder,'''',''''''),oldInit));
set_param(model,'SimulationCommand','update');
for k=1:numel(dest)
    l=get_param(dest(k),'Line');assert(l~=-1&&get_param(l,'SrcPortHandle')==total.Outport(1));
end
for k=1:numel(gainNames)
    assert(strcmp(get_param([s '/' strrep(gainNames{k},'/','//')],'Gain'),oldGains{k}),'P/D gain changed.');
end
assert(strcmp(get_param([s '/YawI Enable'],'Value'),'0'),'Experiment must default to OFF.');
if ~exist(fullfile(project,'saved_data',model),'dir'),mkdir(fullfile(project,'saved_data',model));end
output=fullfile(project,[model '.mdl']);
if exist(output,'file'),copyfile(output,[output '.' datestr(now,'yyyymmdd_HHMMSS') '.bak']);end
save_system(model,output);
fprintf('PASS: YawI diagram; P/D retained; actual Arm gate; limit freeze; dedicated log; Enable=0.\n');
if buildExecutable
    set_param(model,'DownloadToTarget','off');slbuild(model);
    fprintf('PASS: YawI local build. No download, Start or Arm.\n');
end
fprintf('Created %s. YawI is DISABLED. Read YAW_I_TRIAL_VI.md before any actuator test.\n',output);
end

function make_chart(path,script,names,sizes)
r=sfroot;c=r.find('-isa','Stateflow.EMChart','Path',path);assert(numel(c)==1);
c.Script=script;c.ChartUpdate='DISCRETE';c.SampleTime='0.001';
for k=1:numel(names),d=c.find('-isa','Stateflow.Data','Name',names{k});assert(numel(d)==1);d.Props.Array.Size=sizes{k};end
end
function p=ports(path),p=get_param(path,'PortHandles');end
function connect(sys,source,out,dest,in)
a=ports([sys '/' source]);b=ports([sys '/' dest]);
add_line(sys,a.Outport(out),b.Inport(in),'autorouting','on');
end
function column(sys,source,out,dest,in,width,name)
add_block('simulink/Math Operations/Reshape',[sys '/' name], ...
    'OutputDimensionality','Customize','OutputDimensions',sprintf('[%d 1]',width), ...
    'Position',[1600 2500+in*45 1690 2530+in*45]);
connect(sys,source,out,name,1);connect(sys,name,1,dest,in);
end
