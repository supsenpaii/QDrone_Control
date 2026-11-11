function check_pid_diagnostics(buildExecutable, model)
% Check licenses, update diagram, optionally build locally; never Start/Arm.
if nargin == 0, buildExecutable = false; end
if nargin < 2, model = 'Stabilizer_QDrone_048378_PID_Diag'; end
project = fileparts(fileparts(mfilename('fullpath')));
cd(project);
products = {'Control_Toolbox','Simulink_Control_Design','Identification_Toolbox'};
for k = 1:numel(products)
    fprintf('LICENSE %s: %d\n', products{k}, license('test', products{k}));
end
fprintf('pidtune: %s\n', which('pidtune'));
fprintf('PID Tuner: %s\n', which('pidTuner'));
fprintf('System identification tfest: %s\n', which('tfest'));
assert(~bdIsLoaded(model), 'Close the diagnostic model before checking it.');
load_system(fullfile(project, [model '.mdl']));
cleanup = onCleanup(@() close_system(model, 0)); %#ok<NASGU>
controller = [model '/STABILIZER - QDRONE'];
motors = [model '/PHYSICAL PLANT w// LINEAR THRUST'];
if strcmp(model, 'Stabilizer_QDrone_048378_GyroCal')
    check_inputs(motors, 'Gyro Bias Compensation', {'Demux'});
    correction = str2num(get_param([motors '/Gyro Bias Compensation'], 'Bias')); %#ok<ST2NM>
    expectedCorrection = [0.023051257131289856 0.0010684602293397907 0.0025229722902974675];
    assert(numel(correction)==3 && max(abs(correction(:)'-expectedCorrection))<1e-12, ...
        'Unexpected gyro correction.');
    fprintf('PASS: gyro bias correction source and rad/s constants.\n');
end
controlSources = {'PID Diagnostic Time', sprintf('Saturate\nStabilization\nCommand'), ...
    'Select Angular Position', sprintf('Select Angular\nSpeed'), 'Sum1','Sum3', ...
    sprintf('Saturation\nCommand \nAuthority (Nm)'), 'Throttle (N)','Arm (0,1)','Mode (0,1)'};
motorSources = {'PID Diagnostic Time','Product', sprintf('Maximum \nCommand\nAuthority '), ...
    sprintf('Bias - Pulse \nOffset (%%)'), 'Conversion','Arm (0,1)'};
check_inputs(controller, 'PID Diagnostic Signals', controlSources);
check_inputs(motors, 'PID Diagnostic Signals', motorSources);
check_inputs(controller, 'PID Diagnostic Log', {'PID Diagnostic Signals'});
check_inputs(motors, 'PID Diagnostic Log', {'PID Diagnostic Signals'});
check_inputs(controller, 'PID Diagnostic Scope', {'Sum3'});
check_inputs(motors, 'PID Diagnostic Scope', {sprintf('Maximum \nCommand\nAuthority ')});
fprintf('PASS: all diagnostic inputs connected to their expected sources.\n');
assert(strcmp(get_param(model,'FixedStep'),'0.001'));
set_param(model, 'SimulationCommand', 'update');
fprintf('PASS: diagnostic diagram update.\n');
if buildExecutable
    set_param(model, 'DownloadToTarget', 'off');
    slbuild(model);
    fprintf('PASS: local build completed without download or Start.\n');
end
end
function check_inputs(system, destination, sources)
ports = get_param([system '/' destination], 'PortHandles');
assert(numel(ports.Inport)==numel(sources), 'Unexpected diagnostic input count.');
for k = 1:numel(sources)
    line = get_param(ports.Inport(k), 'Line');
    assert(line~=-1, 'Missing connection: %s/%s input %d.', system, destination, k);
    actual = get_param(line, 'SrcPortHandle');
    expected = get_param([system '/' sources{k}], 'PortHandles');
    assert(actual==expected.Outport(1), 'Wrong source: %s/%s input %d.', system, destination, k);
end
end
