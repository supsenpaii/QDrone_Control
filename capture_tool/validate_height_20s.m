folder=fileparts(mfilename('fullpath'));addpath(folder);cd(fileparts(folder));
test_baro_height;
cfg=coder.config('lib');cfg.GenCodeOnly=true;
codegen('-config',cfg,'qdrone_baro_height','-args',{0,0,0,0,false},'-d',fullfile(folder,'codegen_check','height'));
fprintf('PASS: 20-second height estimator C generation.\n');
model='Read_IMU_Height';file=fullfile(fileparts(folder),[model '.slx']);
copyfile(file,[file '.before-20s-' datestr(now,'yyyymmdd_HHMMSS') '.bak']);
load_system(file);
annotations=find_system([model '/Ground Reference Height'],'FindAll','on','Type','annotation');
for k=1:numel(annotations)
 a=get_param(annotations(k),'Object');a.Text=strrep(a.Text,'3 seconds','20 seconds');
end
set_param(model,'SimulationCommand','update');save_system(model);close_system(model,0);
fprintf('PASS: saved 20-second model annotation and diagram update. No target stopped or downloaded.\n');
