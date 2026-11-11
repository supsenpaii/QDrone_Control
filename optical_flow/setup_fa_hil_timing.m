function setup_fa_hil_timing(buildExecutable)
% Measure one periodic invocation of each HIL Read/Write using QUARC timers.
% hil_timing_data rows: time, HIL Read elapsed seconds, HIL Write elapsed seconds.
if nargin<1,buildExecutable=true;end
folder=fileparts(mfilename('fullpath'));project=fileparts(folder);cd(project);addpath(folder);
m='Stabilizer_QDrone_QGC_OF_RYPI_FA';mission='Mission_Server_QDrone_Manual_Flight_FrSky_FA';
load_system(fullfile(project,[m '.mdl']));
assert(strcmp(get_param(m,'SimulationStatus'),'stopped'),'Stop Stabilizer first.');
assert(~bdIsLoaded(mission)||strcmp(get_param(mission,'SimulationStatus'),'stopped'), ...
    'Stop Mission Server first.');
assert(strcmp(get_param(m,'Dirty'),'off'),'Save pending Stabilizer changes first.');
assert(strcmp(get_param(m,'FixedStep'),'0.001'),'Expected the 1 ms model.');
parent=[m '/PHYSICAL PLANT w// LINEAR THRUST'];
timerSource=[m '/STABILIZER - QDRONE/FLIGHT SAFETY/Computation Time'];
logSource=[m '/STABILIZER - QDRONE/To Host File'];
backup=fullfile(project,'_archive','fa_hil_timing');
if ~isfolder(backup),mkdir(backup);end
file=get_param(m,'FileName');
copyfile(file,fullfile(backup,[m '_' datestr(now,'yyyymmdd_HHMMSS') '.mdl']));
try
    readName=sprintf('Battery Level, \nAccelrometer and \nGyroscope');
    wrap(readName,'HIL Read Timed','HIL Read Timer',[400 1100 520 1160]);
    wrap('HIL Write','HIL Write Timed','HIL Write Timer',[400 1200 520 1260]);
    mux=[parent '/HIL Timing Mux'];
    if getSimulinkBlockHandle(mux)==-1
        add_block('simulink/Signal Routing/Mux',mux,'Inputs','2','Position',[600 1130 610 1230]);
    end
    logger=[parent '/HIL Timing Log'];
    if getSimulinkBlockHandle(logger)==-1
        add_block(logSource,logger,'Position',[690 1140 850 1220]);
    end
    set_param(logger,'file_name','saved_data/%m/HILTiming_%{char(datetime)}.mat', ...
        'variable_name','hil_timing_data','decimation','1','sample_time','0.001', ...
        'file_format','MAT-file v.5','options','No header or footer','append','off');
    wire('HIL Read Timer',2,'HIL Timing Mux',1);
    wire('HIL Write Timer',2,'HIL Timing Mux',2);
    wire('HIL Timing Mux',1,'HIL Timing Log',1);
    set_param(m,'DownloadToTarget','off');set_param(m,'SimulationCommand','update');
catch failure
    % Discard an incomplete instrumentation edit and reload the user's saved model.
    close_system(m,0);load_system(file);rethrow(failure);
end
save_system(m);
fprintf('PASS: periodic 1 kHz HIL Read/Write timing with checked external port mappings.\n');
if buildExecutable
    slbuild(m);
    fprintf('PASS: HIL timing local build. Download, Start and Arm remain explicit.\n');
end

    function wrap(originalName,wrapperName,timerName,position)
        wrapper=[parent '/' wrapperName];
        if getSimulinkBlockHandle(wrapper)==-1
            original=[parent '/' originalName];h=get_param(original,'Handle');
            ports=get_param(h,'PortHandles');inputs=zeros(size(ports.Inport));
            outputs=cell(size(ports.Outport));priority=get_param(h,'Priority');
            for k=1:numel(inputs)
                line=get_param(ports.Inport(k),'Line');assert(line~=-1,'Disconnected HIL input.');
                inputs(k)=get_param(line,'SrcPortHandle');
            end
            for k=1:numel(outputs),outputs{k}=destinations(ports.Outport(k));end
            before=find_system(parent,'SearchDepth',1,'BlockType','SubSystem');
            open_system(parent);
            Simulink.BlockDiagram.createSubsystem(h);
            after=find_system(parent,'SearchDepth',1,'BlockType','SubSystem');
            created=setdiff(after,before);assert(numel(created)==1,'Expected one new HIL subsystem.');
            set_param(created{1},'Name',wrapperName);
            newPorts=get_param(wrapper,'PortHandles');
            assert(numel(newPorts.Inport)==numel(inputs)&&numel(newPorts.Outport)==numel(outputs), ...
                'HIL wrapper port count changed.');
            for k=1:numel(inputs)
                line=get_param(newPorts.Inport(k),'Line');
                assert(line~=-1&&get_param(line,'SrcPortHandle')==inputs(k),'HIL input order changed.');
            end
            for k=1:numel(outputs)
                assert(isequal(destinations(newPorts.Outport(k)),outputs{k}),'HIL output order changed.');
            end
            trigger=[wrapper '/Timing Trigger'];
            add_block('simulink/Ports & Subsystems/Trigger',trigger,'Position',[40 10 60 30]);
            set_param(trigger,'TriggerType','function-call','SampleTimeType','periodic','SampleTime','0.001');
            set_param(wrapper,'TreatAsAtomicUnit','on');
            timer=[parent '/' timerName];
            add_block(timerSource,timer,'Position',position);
            set_param(timer,'use_model','off','use_cpu_time','off','sample_time','0.001','Priority',priority);
        end
        timer=[parent '/' timerName];trigger=[wrapper '/Timing Trigger'];
        assert(getSimulinkBlockHandle(timer)~=-1&&getSimulinkBlockHandle(trigger)~=-1, ...
            'Incomplete existing HIL timing wrapper.');
        p=get_param(timer,'PortHandles');w=get_param(wrapper,'PortHandles');
        assert(numel(p.Outport)==2&&numel(w.Trigger)==1,'Unexpected QUARC function-call timer ports.');
        existing=get_param(w.Trigger(1),'Line');
        if existing==-1
            add_line(parent,p.Outport(1),w.Trigger(1),'autorouting','on');
        else
            assert(get_param(existing,'SrcPortHandle')==p.Outport(1),'Unexpected HIL trigger source.');
        end
    end

    function result=destinations(port)
        line=get_param(port,'Line');result=[];
        if line~=-1
            result=get_param(line,'DstPortHandle');result=sort(result(result~=-1));result=result(:);
        end
    end

    function wire(src,sp,dst,dp)
        a=get_param([parent '/' src],'PortHandles');b=get_param([parent '/' dst],'PortHandles');
        line=get_param(b.Inport(dp),'Line');
        if line==-1
            add_line(parent,a.Outport(sp),b.Inport(dp),'autorouting','on');
        else
            assert(get_param(line,'SrcPortHandle')==a.Outport(sp),'Unexpected HIL timing connection.');
        end
    end
end
