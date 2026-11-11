function fix_fa_frame_freshness(buildExecutable)
% Publish a timestamp and flow together only when the camera reports new data.
if nargin<1,buildExecutable=false;end
folder=fileparts(mfilename('fullpath'));project=fileparts(folder);
addpath(folder);cd(project);
m='Stabilizer_QDrone_QGC_OF_RYPI_FA';
load_system(fullfile(project,[m '.mdl']));
assert(strcmp(get_param(m,'SimulationStatus'),'stopped'),'Stop Stabilizer first.');
assert(strcmp(get_param(m,'Dirty'),'off'),'Save pending Stabilizer changes first.');
s=[m '/STABILIZER - QDRONE/Optical Flow Monitor'];
rate=[s '/FA Frame Base Rate'];
p=get_param(rate,'PortHandles');line=get_param(p.Inport(1),'Line');
assert(line~=-1,'Frame packet is disconnected.');
source=get_param(get_param(line,'SrcPortHandle'),'Parent');
block=[s '/FA Fresh Frame Packet'];
assert(any(strcmp(source,{[s '/FA Frame Packet'],block})), ...
    'Unexpected frame packet source.');
backup=fullfile(project,'_archive','fa_frame_freshness');
if ~isfolder(backup),mkdir(backup);end
copyfile(get_param(m,'FileName'),fullfile(backup, ...
    [m '_' datestr(now,'yyyymmdd_HHMMSS') '.mdl']));
if getSimulinkBlockHandle(block)==-1
    add_block('simulink/User-Defined Functions/MATLAB Function',block, ...
        'Position',[1130 730 1330 820]);
    root=sfroot;
    chart=root.find('-isa','Stateflow.EMChart','Path',block);
    chart.Script=sprintf(['function packet=holdFrame(candidate,fresh)\n%%#codegen\n' ...
        'persistent held\nif isempty(held),held=zeros(9,1);end\n' ...
        'if candidate(1)<held(1),held=zeros(9,1);end\n' ...
        'if fresh,held=candidate;end\npacket=held;\nend\n']);
    chart.ChartUpdate='DISCRETE';chart.SampleTime='0.033';
    names={'candidate','fresh','packet'};sizes={'[9 1]','1','[9 1]'};
    for k=1:numel(names)
        d=chart.find('-isa','Stateflow.Data','Name',names{k});
        assert(numel(d)==1);d.Props.Array.Size=sizes{k};
    end
end
wire('FA Frame Packet',1,'FA Fresh Frame Packet',1);
wire('Down Camera',2,'FA Fresh Frame Packet',2);
if strcmp(source,[s '/FA Frame Packet'])
    delete_line(s,'FA Frame Packet/1','FA Frame Base Rate/1');
end
wire('FA Fresh Frame Packet',1,'FA Frame Base Rate',1);
set_param(m,'DownloadToTarget','off');
set_param(m,'SimulationCommand','update');save_system(m);
fprintf('PASS: frame timestamp held with flow until a new camera frame.\n');
if buildExecutable
    slbuild(m);
    fprintf('PASS: Stabilizer local build. Download and Start remain explicit.\n');
end
    function wire(src,sp,dst,dp)
        a=get_param([s '/' src],'PortHandles');b=get_param([s '/' dst],'PortHandles');
        existing=get_param(b.Inport(dp),'Line');
        if existing==-1
            add_line(s,a.Outport(sp),b.Inport(dp),'autorouting','on');
        else
            assert(get_param(existing,'SrcPortHandle')==a.Outport(sp), ...
                'Unexpected input to %s.',dst);
        end
    end
end
