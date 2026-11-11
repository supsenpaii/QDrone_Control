project=fileparts(fileparts(mfilename('fullpath')));
folder=fullfile(project,'optical_flow');addpath(folder);cd(project);
test_qdrone_of;
f=uint8(ones(480,640)*80);flow=zeros(8,1);
a=qdrone_of_overlay(f,flow);assert(isequal(size(a),[480 640 3]) && isa(a,'uint8'));assert(a(1,1,1)==255);
flow(1:4)=[2;-1;0.5;1];b=qdrone_of_overlay(f,flow);assert(b(1,1,2)==255);
assert(isequal(squeeze(b(400,100,:)),uint8([80;80;80])));
fprintf('PASS: overlay dimensions, invalid/valid colors and unmodified background.\n');
original='Stabilizer_QDrone_048378_OF';model='Stabilizer_QDrone_048378_OF_Ready';
output=fullfile(project,[model '.slx']);
if exist(output,'file'),copyfile(output,[output '.' datestr(now,'yyyymmdd_HHMMSS') '.bak']);end
load_system(fullfile(project,[original '.slx']));
save_system(original,output);load_system(output);
set_param(model,'InitFcn',strrep(get_param(model,'InitFcn'),original,model));
s=[model '/STABILIZER - QDRONE/Optical Flow Monitor'];
old=[s '/Log Rate Transition'];if getSimulinkBlockHandle(old)~=-1,delete_block(old);end
rates={'Gyro Log Rate','Attitude Log Rate'};
for k=1:2
    p=[s '/' rates{k}];
    if getSimulinkBlockHandle(p)==-1
        add_block('simulink/Signal Attributes/Rate Transition',p,'Position',[175 260+70*(k-1) 225 300+70*(k-1)]);
    end
    set_param(p,'OutPortSampleTime','0.033');
end
connections={'Gyro body rad_s','Gyro Log Rate',1;'Attitude rad','Attitude Log Rate',1; ...
    'Gyro Log Rate','Log Signals',3;'Attitude Log Rate','Log Signals',4;'Log Signals','Flow Log',1};
for k=1:size(connections,1)
    src=get_param([s '/' connections{k,1}],'PortHandles');dst=get_param([s '/' connections{k,2}],'PortHandles');
    input=dst.Inport(connections{k,3});line=get_param(input,'Line');
    if line~=-1
        if get_param(line,'SrcPortHandle')==src.Outport(1),continue;end
        delete_line(line);
    end
    add_line(s,src.Outport(1),input,'autorouting','on');
end
set_param([s '/Flow Log'],'sample_time','0.033');
% Remove dangling branches left by earlier manual repairs, retaining good wires.
lines=find_system(s,'FindAll','on','SearchDepth',1,'Type','line');
for k=1:numel(lines)
    if ~ishandle(lines(k)),continue;end
    children=get_param(lines(k),'LineChildren');dst=get_param(lines(k),'DstPortHandle');
    if isempty(children) && (isempty(dst) || all(dst==-1)),delete_line(lines(k));end
end
chartPath=[s '/Optical Flow'];sf=sfroot;chart=sf.find('-isa','Stateflow.EMChart','Path',chartPath);
chart.Script=sprintf(['function [y,rgb]=run(image,fresh,t)\n%%#codegen\n' ...
    'y=qdrone_of_step(image,fresh,t,false);\nrgb=qdrone_of_overlay(image,y);\nend\n']);
viewer=[s '/Optical Flow Camera feed'];vp=get_param(viewer,'PortHandles');wire=get_param(vp.Inport(1),'Line');
if wire~=-1,delete_line(wire);end
add_line(s,'Optical Flow/2','Optical Flow Camera feed/1','autorouting','on');
set_param([viewer '/Video Display'],'Tag',[viewer '/Video Display']);
% Give each diagnostic a name; do not mix the age sentinel into motion autoscale.
names={'Image motion px per frame','Quality 0 to 1','Flow valid 0 or 1','Frame interval seconds'};
indices={'[1 2]','3','4','5'};
for k=1:4
    selector=[s '/Select ' names{k}];display=[s '/' names{k}];
    if getSimulinkBlockHandle(selector)==-1
        add_block('simulink/Signal Routing/Selector',selector,'NumberOfDimensions','1', ...
            'IndexOptionArray',{'Index vector (dialog)'},'Indices',indices{k}, ...
            'InputPortWidth','8','Position',[730 30+75*(k-1) 770 60+75*(k-1)]);
        add_block('simulink/Sinks/Display',display,'Position',[810 25+75*(k-1) 980 65+75*(k-1)]);
        add_line(s,'Optical Flow/1',['Select ' names{k} '/1'],'autorouting','on');
        add_line(s,['Select ' names{k} '/1'],[names{k} '/1']);
    end
end
% Keep original eight-value Flow Status and raw log for compatibility.
annotation=Simulink.Annotation(s,sprintf(['GREEN = valid image flow; RED = invalid/stale\n' ...
    'Arrow = IMAGE motion (8x display scale), not drone direction\n' ...
    'Quality bar = fraction of agreeing patches\n' ...
    'This model does NOT hold position automatically.']));annotation.Position=[730 365];
set_param(model,'SimulationCommand','update');
warningState=lastwarn;fprintf('Last diagram warning: %s\n',warningState);
save_system(model);fprintf('PASS: updated diagram and saved %s\n',output);
% Verify standalone estimator and overlay can generate C code, without hardware.
try
    cfg=coder.config('lib');cfg.GenCodeOnly=true;
    codegen('-config',cfg,'qdrone_of_step','-args',{zeros(480,640,'uint8'),false,0,false},'-d',fullfile(folder,'codegen_check','flow'));
    codegen('-config',cfg,'qdrone_of_overlay','-args',{zeros(480,640,'uint8'),zeros(8,1)},'-d',fullfile(folder,'codegen_check','overlay'));
    fprintf('PASS: C code generation for estimator and overlay.\n');
catch cause
    fprintf('CODEGEN CHECK FAILED: %s\n',getReport(cause,'extended','hyperlinks','off'));rethrow(cause);
end
files=dir(fullfile(project,'saved_data',original,'OpticalFlow_*.mat'));[~,order]=sort([files.datenum],'descend');
for j=order
    if files(j).bytes==0,continue;end
    try
        data=load(fullfile(files(j).folder,files(j).name));v=data.optical_flow_data;
        if size(v,1)~=16,continue;end
        mask=v(6,:)>0.5;good=sum(mask);n=size(v,2);
        fprintf('LOG %s: samples=%d, valid=%d (%.1f%%), duration=%.3fs\n',files(j).name,n,good,100*good/max(1,n),v(1,end)-v(1,1));
        if good>0
            fprintf('Valid flow du=[%.3f %.3f] dv=[%.3f %.3f] px/frame; median dt=%.4f s; median quality=%.3f\n', ...
                min(v(3,mask)),max(v(3,mask)),min(v(4,mask)),max(v(4,mask)),median(v(7,mask)),median(v(5,mask)));
        end
        break;
    catch cause,fprintf('Log summary: %s\n',cause.message);end
end
close_system(model,0);
fprintf('DONE. No target downloaded, started or armed.\n');
