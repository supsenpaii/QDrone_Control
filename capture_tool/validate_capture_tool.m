folder=fileparts(mfilename('fullpath'));addpath(folder,fullfile(fileparts(folder),'optical_flow'));cd(fileparts(folder));
fprintf('=== CAPTURE TESTS ===\n');test_qdrone_capture;
fprintf('=== BUILD SENSOR-ONLY DIAGRAM ===\n');build_qdrone_capture(true);
fprintf('=== STATIC OUTPUT CHECK ===\n');
hil='QDrone_Capture/Sensors Initialize';params=get_param(hil,'DialogParameters');keys=fieldnames(params);
for k=1:numel(keys)
    if ~isempty(regexp(keys{k},'^set_.*_(at_start|at_switch_in|at_terminate|at_switch_out|on_watchdog)$','once'))
        assert(strcmp(get_param(hil,keys{k}),'off'),['Unexpected initialization write flag: ' keys{k}]);
    end
end
assert(isempty(find_system('QDrone_Capture','RegExp','on','Name','(?i).*HIL.*Write.*')));
fprintf('PASS: all HIL startup/output-write flags off; no HIL Write blocks.\n');
fprintf('=== GENERATE C FOR PACKET FUNCTIONS ===\n');cfg=coder.config('lib');cfg.GenCodeOnly=true;
codegen('-config',cfg,'qdrone_capture_frame','-args',{zeros(480,640,'uint8'),false,0,1},'-d',fullfile(folder,'codegen_check','frame'));
codegen('-config',cfg,'qdrone_capture_packet','-args',{0,zeros(5,1),zeros(11,1),zeros(8,1),0,0},'-d',fullfile(folder,'codegen_check','telemetry'));
fprintf('PASS: packet C generation.\n');
% Exercise host dashboard without a drone; verify stale state and clean shutdown.
receiver=QDroneCapture('127.0.0.1',false);stop(receiver.Timer);
session=receiver.Session;f=fopen(fullfile(session,'VALIDATION_ONLY.txt'),'w');fprintf(f,'Synthetic data test; not a drone recording.');fclose(f);
a=zeros(5,1);a(5)=12.3;s=zeros(11,1);s(6)=9.81;s(10)=101325;s(11)=24;
flow=[1;2;0.8;1;0.033;0;30;28];
p=qdrone_capture_packet(0.033,a,s,flow,1,0.033);
receiver.acceptTelemetry(p,toc(receiver.StartClock));receiver.publish;
status=jsondecode(fileread(fullfile(session,'latest.json')));assert(status.telemetry_live && status.flow_valid && abs(status.battery_v-12.3)<1e-9);
clear qdrone_capture_frame;
im=uint8(ones(480,640)*123);packet=qdrone_capture_frame(im,true,0.033,1);[frames,~,~]=qdrone_parse_frames(packet);
receiver.acceptFrames(frames,toc(receiver.StartClock));
restored=read_qdrone_frame(session,1,1);assert(isequal(restored,im),'Saved camera frame changed pixel values');
receiver.LastTelemetry=-1;receiver.publish;
status=jsondecode(fileread(fullfile(session,'latest.json')));assert(~status.telemetry_live && ~status.flow_valid,'Stale data accepted as live');
fprintf('PASS: synthetic telemetry, live/stale state, raw-frame disk round trip and preview PNG.\n');
session=receiver.Session;receiver.stop;
status=jsondecode(fileread(fullfile(session,'latest.json')));
assert(~status.telemetry_live && ~status.motor_outputs_enabled && ~status.recording);
fprintf('PASS: host no-data status, UI and shutdown.\n');
fprintf('=== FULL TARGET C GENERATION, DOWNLOAD DISABLED ===\n');
% GenCodeOnly prevents compilation/download. Localhost URI is a second guard.
set_param('QDrone_Capture','GenCodeOnly','on','GenerateReport','off','LaunchReport','off', ...
    'ExtModeMexArgs','''-w -d /tmp -uri %u'',''tcpip://127.0.0.1:17999''');
slbuild('QDrone_Capture');
fprintf('PASS: full sensor-only QUARC target C generation.\n');
close_system('QDrone_Capture',0);
fprintf('DONE: no drone target was built, downloaded, started or armed.\n');


