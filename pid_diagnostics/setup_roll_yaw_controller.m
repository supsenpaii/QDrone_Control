function model=setup_roll_yaw_controller(buildExecutable)
% Yaw-only I with allocator anti-windup; legacy model/log schema retained.
% Disabled by default; preserves P/D, camera, QGC, watchdog and motor Arm.
% No download, Start or Arm. MATLAB wrapper test runs before model creation.
if nargin<1,buildExecutable=false;end
folder=fileparts(mfilename('fullpath'));project=fileparts(folder);
cd(project);addpath(folder,fullfile(project,'optical_flow'));
test_roll_yaw_controller;
source='Stabilizer_QDrone_QGC_OF_YawI';model='Stabilizer_QDrone_QGC_OF_RYPI';
assert(~bdIsLoaded(model),'Close %s before setup.',model);
assert(~bdIsLoaded(source) || strcmp(get_param(source,'Dirty'),'off'), ...
    'Save the source YawI model first; setup copies the saved controller.');
pending=fullfile(project,'qgc_bridge','pending');if ~exist(pending,'dir'),mkdir(pending);end
f=fopen(fullfile(project,[source '.mdl']),'r','n','windows-1252');assert(f~=-1,'Source YawI model missing.');
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
% Remove old experiment only, retaining its summed output and all PD consumers.
remove={};
for system={s,p}
    list=find_system(system{1},'SearchDepth',1,'Type','Block');
    for k=1:numel(list)
        name=get_param(list{k},'Name');
        if strncmp(name,'YawI ',5) || strcmp(name,'Bounded Yaw Integral'),remove{end+1}=list{k};end %#ok<AGROW>
    end
end
before=connections(model);keep=true(size(before));
for k=1:numel(before),keep(k)=~any(strcmp(before(k).src,remove)) && ~any(strcmp(before(k).dst,remove));end
before=before(keep);
oldTotal=[s '/PD Plus Yaw I'];total=oldTotal;
for k=1:numel(remove),remove_block_clean(remove{k});end
for k=1:numel(before)
    if strcmp(before(k).src,oldTotal),before(k).src=total;end
    if strcmp(before(k).dst,oldTotal),before(k).dst=total;end
end
% Derive the saturation feedback from the actual saved allocator matrix.
mixer=str2num(get_param([p '/Constant1'],'Value')); %#ok<ST2NM>
assert(isequal(size(mixer),[4 4]) && all(isfinite(mixer(:))) && rcond(mixer)>1e-8, ...
    'Expected a finite invertible 4x4 mixer.');
inverse=inv(mixer);assert(max(max(abs(mixer*inverse-eye(4))))<1e-9);
upper=str2num(get_param([p '/' sprintf('Maximum \nCommand\nAuthority ')],'UpperLimit')); %#ok<ST2NM>
lower=str2num(get_param([p '/' sprintf('Maximum \nCommand\nAuthority ')],'LowerLimit')); %#ok<ST2NM>
assert(~isempty(upper)&&~isempty(lower)&&all(upper(:)==1)&&all(lower(:)==0), ...
    'Anti-windup assumes the source motor limiter is normalized 0..1.');

mp=ports([p '/Product1']);armLine=get_param(mp.Inport(2),'Line');assert(armLine~=-1);
actualArm=get_param(armLine,'SrcPortHandle');
add_block('simulink/Signal Routing/Goto',[p '/RYPI Arm Tap'],'GotoTag','RYPI_ActualArm', ...
    'TagVisibility','global','Position',[2300 1900 2420 1920]);
gp=ports([p '/RYPI Arm Tap']);add_line(p,actualArm,gp.Inport(1),'autorouting','on');
add_block('simulink/Signal Routing/Goto',[p '/RYPI Mixer Tap'],'GotoTag','RYPI_RawMotors', ...
    'TagVisibility','global','Position',[2300 1950 2420 1970]);
connect(p,'Product',1,'RYPI Mixer Tap',1);
add_block('simulink/Signal Routing/From',[s '/RYPI Actual Arm'],'GotoTag','RYPI_ActualArm', ...
    'Position',[1400 1900 1540 1920]);
add_block('simulink/Signal Routing/From',[s '/RYPI Raw Motors'],'GotoTag','RYPI_RawMotors', ...
    'Position',[1400 1950 1540 1970]);
add_block('simulink/Sources/Constant',[s '/RYPI Inverse Mixer'],'Value',mat2str(inverse,17), ...
    'Position',[1400 2000 1580 2040]);
aw=[s '/RYPI Allocator Feedback'];
add_block('simulink/User-Defined Functions/MATLAB Function',aw,'Position',[1700 1950 1900 2070]);
make_chart(aw,sprintf(['function residual=feedback(motor,rawTorque,limitedTorque,inverseMixer)\n%%#codegen\n' ...
    'residual=qdrone_ry_aw_step(motor,rawTorque,limitedTorque,inverseMixer);\nend\n']), ...
    {'motor','rawTorque','limitedTorque','inverseMixer','residual'}, ...
    {'[4 1]','[3 1]','[3 1]','[4 4]','[3 1]'});
column(s,'RYPI Raw Motors',1,'RYPI Allocator Feedback',1,4,'RYPI Motor Column');
column(s,'Sum4',1,'RYPI Allocator Feedback',2,3,'RYPI Raw Torque Column');
column(s,sprintf('Saturation\nCommand \nAuthority (Nm)'),1,'RYPI Allocator Feedback',3,3,'RYPI Limited Torque Column');
connect(s,'RYPI Inverse Mixer',1,'RYPI Allocator Feedback',4);
add_block('simulink/Discrete/Unit Delay',[s '/RYPI Previous Residual'], ...
    'SampleTime','0.001','InitialCondition','[0;0;0]','Position',[1950 1980 2010 2020]);
connect(s,'RYPI Allocator Feedback',1,'RYPI Previous Residual',1);
% Reserved roll slots are zero. Yaw uses the previously configured parameters.
config='[0;0.01;0;0.015;0;0.003;0;1]';
add_block('simulink/Sources/Constant',[s '/RYPI Enable Roll Yaw'],'Value','[0;0]', ...
    'Position',[1400 2100 1580 2140]);
add_block('simulink/Sources/Constant',[s '/RYPI Configuration'],'Value',config, ...
    'Position',[1400 2160 1600 2210]);
add_block('simulink/Sources/Constant',[s '/RYPI Reset'],'Value','0', ...
    'Position',[1400 2250 1540 2280]);
controller=[s '/Yaw Integral Controller'];
add_block('simulink/User-Defined Functions/MATLAB Function',controller,'Position',[2100 1870 2370 2250]);
make_chart(controller,sprintf(['function [term,diagnostic]=step(errorRate,angles,gyro,residual,arm,throttle,mode,enable,config,reset)\n%%#codegen\n' ...
    '[term,diagnostic]=qdrone_ry_pi_step(errorRate,angles,gyro,residual,arm,throttle,mode,enable,config,reset);\nend\n']), ...
    {'errorRate','angles','gyro','residual','enable','config','term','diagnostic'}, ...
    {'[3 1]','[3 1]','[3 1]','[3 1]','[2 1]','[8 1]','[3 1]','[10 1]'});
sources={'Sum2','Select Angular Position',sprintf('Select Angular\nSpeed'), ...
    'RYPI Previous Residual','RYPI Actual Arm','Throttle (N)','Mode (0,1)', ...
    'RYPI Enable Roll Yaw','RYPI Configuration','RYPI Reset'};
for k=1:numel(sources)
    if k<=3,column(s,sources{k},1,'Yaw Integral Controller',k,3,sprintf('RYPI Input %d Column',k));
    else,connect(s,sources{k},1,'Yaw Integral Controller',k);end
end
add_block('simulink/Math Operations/Reshape',[s '/RYPI Torque 1D'], ...
    'OutputDimensionality','1-D array','Position',[2410 1870 2480 1900]);
connect(s,'Yaw Integral Controller',1,'RYPI Torque 1D',1);
connect(s,'RYPI Torque 1D',1,'PD Plus Roll Yaw I',2);
add_block('simulink/Signal Routing/Selector',[s '/RYPI Select Status'],'NumberOfDimensions','1', ...
    'IndexOptionArray',{'Index vector (dialog)'},'Indices','[1 2]','InputPortWidth','10', ...
    'Position',[2420 2050 2480 2090]);
connect(s,'Yaw Integral Controller',2,'RYPI Select Status',1);
add_block('simulink/Sinks/Display',[s '/RYPI Status Roll Yaw'],'Position',[2520 2050 2680 2100]);
connect(s,'RYPI Select Status',1,'RYPI Status Roll Yaw',1);

add_block('simulink/Sources/Digital Clock',[s '/RYPI Time'],'sampleTime','0.001', ...
    'Position',[2400 2200 2480 2230]);
add_block('simulink/Signal Routing/Mux',[s '/RYPI Telemetry'],'Inputs','19', ...
    'Position',[2750 1900 2755 2550]);
logs={'RYPI Time','Sum2',sprintf('Select Angular\nSpeed'),'Select Angular Position','Sum3', ...
    'RYPI Torque 1D','PD Plus Roll Yaw I','Sum4',sprintf('Saturation\nCommand \nAuthority (Nm)'), ...
    'RYPI Previous Residual','Yaw Integral Controller','RYPI Actual Arm','Throttle (N)', ...
    'Mode (0,1)','RYPI Enable Roll Yaw','RYPI Configuration','RYPI Raw Motors', ...
    sprintf('Saturate\nStabilization\nCommand'),'RYPI Reset'};
for k=1:numel(logs),out=1;if k==11,out=2;end;connect(s,logs{k},out,'RYPI Telemetry',k);end
add_block('simulink/Signal Attributes/Signal Specification',[s '/RYPI Log Width'], ...
    'Dimensions','59','Position',[2800 2190 2860 2230]);
connect(s,'RYPI Telemetry',1,'RYPI Log Width',1);
add_block([s '/To Host File'],[s '/RYPI Log'],'Position',[2910 2180 3070 2250]);
set_param([s '/RYPI Log'],'file_name','saved_data/%m/RollYawPI_%{char(datetime)}.mat', ...
    'variable_name','roll_yaw_pi_data','decimation','2','sample_time','0.001');
connect(s,'RYPI Log Width',1,'RYPI Log',1);
oldInit=get_param(model,'InitFcn');
set_param(model,'InitFcn',sprintf('addpath(''%s'');\n%s',strrep(folder,'''',''''''),oldInit));
% Capture the real chart diagnostic; MATLAB's thrown exception often only
% says "Errors occurred during parsing" without the source error.
diagFile=fullfile(folder,'roll_yaw_diagnostics.log');
sldiagviewer.diary(diagFile);
diagCleanup=onCleanup(@()sldiagviewer.diary('off')); %#ok<NASGU>
try
    set_param(model,'SimulationCommand','update');
catch cause
    sldiagviewer.diary('off');
    fprintf(2,'%s\n',getReport(cause,'extended','hyperlinks','off'));
    if isfile(diagFile),fprintf(2,'DIAGNOSTIC VIEWER LOG:\n%s\n',fileread(diagFile));end
    rethrow(cause);
end
for k=1:numel(before)
    dp=ports(before(k).dst);line=get_param(dp.Inport(before(k).dstPort),'Line');
    assert(line~=-1,'An original connection disappeared.');sp=get_param(line,'SrcPortHandle');
    assert(strcmp(get_param(sp,'Parent'),before(k).src)&&get_param(sp,'PortNumber')==before(k).srcPort, ...
        'An original PD/plant/telemetry source changed.');
end
for k=1:numel(gainNames)
    assert(strcmp(get_param([s '/' strrep(gainNames{k},'/','//')],'Gain'),oldGains{k}),'P/D gain changed.');
end
tp=ports(total);ip=ports([s '/RYPI Torque 1D']);line=get_param(tp.Inport(2),'Line');
assert(get_param(line,'SrcPortHandle')==ip.Outport(1),'Integral not connected to the original torque path.');
assert(strcmp(get_param([s '/RYPI Enable Roll Yaw'],'Value'),'[0;0]'));
if ~exist(fullfile(project,'saved_data',model),'dir'),mkdir(fullfile(project,'saved_data',model));end
output=fullfile(project,[model '.mdl']);
if exist(output,'file'),copyfile(output,[output '.' datestr(now,'yyyymmdd_HHMMSS') '.bak']);end
save_system(model,output);
fprintf('PASS: RYPI diagram, original PD/plant connections, mixer feedback, log width=59, Enable=[0;0].\n');
if buildExecutable
    set_param(model,'DownloadToTarget','off');slbuild(model);
    fprintf('PASS: RYPI Linux x64 local build. No download, Start or Arm.\n');
end
fprintf('Created %s. Integral disabled. Read ROLL_YAW_PI_VI.md before actuator tests.\n',output);
end

function remove_block_clean(path)
ph=get_param(path,'PortHandles');sys=get_param(path,'Parent');
% Delete the exact incoming branch, not the source line shared with PD sinks.
for k=1:numel(ph.Inport)
    line=get_param(ph.Inport(k),'Line');
    if line==-1,continue;end
    src=get_param(line,'SrcPortHandle');
    if src~=-1
        name=get_param(get_param(src,'Parent'),'Name');
        from=[strrep(name,'/','//') '/' num2str(get_param(src,'PortNumber'))];
        to=[strrep(get_param(path,'Name'),'/','//') '/' num2str(k)];
        delete_line(sys,from,to);
    elseif ishandle(line),delete_line(line);end
end
for k=1:numel(ph.Outport)
    line=get_param(ph.Outport(k),'Line');
    if line~=-1 && ishandle(line),delete_line(line);end
end
delete_block(path);
end
function make_chart(path,script,names,sizes)
r=sfroot;c=r.find('-isa','Stateflow.EMChart','Path',path);assert(numel(c)==1);
c.Script=script;c.ChartUpdate='DISCRETE';c.SampleTime='0.001';
for k=1:numel(names),d=c.find('-isa','Stateflow.Data','Name',names{k});assert(numel(d)==1);d.Props.Array.Size=sizes{k};end
end
function p=ports(path),p=get_param(path,'PortHandles');end
function connect(sys,source,out,dest,in)
a=ports([sys '/' source]);b=ports([sys '/' dest]);add_line(sys,a.Outport(out),b.Inport(in),'autorouting','on');
end
function column(sys,source,out,dest,in,width,name)
add_block('simulink/Math Operations/Reshape',[sys '/' name], ...
    'OutputDimensionality','Customize','OutputDimensions',sprintf('[%d 1]',width), ...
    'Position',[1600 2650+in*45 1690 2680+in*45]);
connect(sys,source,out,name,1);connect(sys,name,1,dest,in);
end
function result=connections(model)
result=struct('src',{},'srcPort',{},'dst',{},'dstPort',{});
blocks=find_system(model,'FollowLinks','off','LookUnderMasks','none','Type','Block');
for k=1:numel(blocks)
    p=ports(blocks{k});
    for j=1:numel(p.Inport)
        l=get_param(p.Inport(j),'Line');if l==-1,continue;end
        sp=get_param(l,'SrcPortHandle');if sp==-1,continue;end
        result(end+1)=struct('src',get_param(sp,'Parent'),'srcPort',get_param(sp,'PortNumber'), ...
            'dst',blocks{k},'dstPort',j); %#ok<AGROW>
    end
end
end
