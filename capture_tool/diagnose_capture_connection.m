cd(fileparts(fileparts(mfilename('fullpath'))));
load_system('QDrone_Capture.slx');cfg=getActiveConfigSet('QDrone_Capture');
fprintf('ACTIVE CONFIG: %s\n',cfg.Name);
fprintf('SYSTEM TARGET: %s\n',get_param(cfg,'SystemTargetFile'));
fprintf('MODEL URI: %s\n',qc_get_model_uri(cfg));
fprintf('TARGET URI: %s\n',qc_get_target_uri(cfg));
fprintf('MODEL ARGS: %s\n',qc_get_model_arguments(cfg));
try
 models=qc_get_loaded_models('tcpip://192.168.2.52:17000');
 fprintf('LOADED MODELS ON DRONE:\n');disp(models);
catch cause
 fprintf('TARGET QUERY ERROR: %s\n',cause.message);
end
fprintf('Read-only diagnostics complete. No target started or stopped.\n');
