function setup_mission_estop_displays
% Add passive diagnostics to the existing FA Mission model.
project=fileparts(fileparts(mfilename('fullpath')));
model='Mission_Server_QDrone_Manual_Flight_FrSky_FA';
load_system(fullfile(project,[model '.mdl']));
assert(strcmp(get_param(model,'SimulationStatus'),'stopped'),'Stop Mission Server FA first.');
assert(strcmp(get_param(model,'Dirty'),'off'),'Save pending Mission model changes first.');
file=get_param(model,'FileName');
backupFolder=fullfile(project,'_archive','mission_estop_displays');
if ~isfolder(backupFolder),mkdir(backupFolder);end
copyfile(file,fullfile(backupFolder,[model '_' datestr(now,'yyyymmdd_HHMMSS') '.mdl']));
system=[model '/MISSION SERVER QDRONE MANUAL FLIGHT/JOYSTICK/FrSKYMapping'];
sources={'E-Stop Selector','Threshold','Arm selector'};
names={'EStop Raw Value','Emergency Abort Value','Arm Value'};
for k=1:numel(sources)
    display=[system '/' names{k}];
    if getSimulinkBlockHandle(display)==-1
        add_block('simulink/Sinks/Display',display,'Position',[450 -180+55*k 550 -150+55*k], ...
            'Decimation','1');
        sourcePorts=get_param([system '/' sources{k}],'PortHandles');
        displayPorts=get_param(display,'PortHandles');
        add_line(system,sourcePorts.Outport(1),displayPorts.Inport(1),'autorouting','on');
    end
end
set_param(model,'SimulationCommand','update');
save_system(model);open_system(system);
fprintf('PASS: E-stop/Abort/Arm displays added. Build and Download Mission Server FA next.\n');
end
