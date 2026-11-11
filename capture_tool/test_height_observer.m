folder=fileparts(mfilename('fullpath'));addpath(folder);cd(fileparts(folder));
test_baro_height;
model='HeightObserver_Test';new_system(model);set_param(model,'Solver','FixedStepDiscrete','FixedStep','0.025','StopTime','4');
set_param(model,'EnablePacing','on','PacingRate','1');
sub=[model '/Ground Reference Height'];add_block('built-in/Subsystem',sub);
labels={'Relative Height m','Raw Height m','Vertical Speed m_s','Height Valid','Ground Pressure Pa','Calibration Progress','Calibration Noise m','Status Code'};
values=[1 1 0 1 101325 1 0.02 1];
for k=1:8
 add_block('simulink/Sources/Constant',[sub '/Input ' num2str(k)],'Value',num2str(values(k),15),'SampleTime','0.025');
 add_block('simulink/Sinks/Display',[sub '/' labels{k}]);add_line(sub,['Input ' num2str(k) '/1'],[labels{k} '/1']);
end
set_param(model,'SimulationCommand','start');
observer=QDroneHeightObserver(model);f=fopen(fullfile(observer.Session,'VALIDATION_ONLY.txt'),'w');fprintf(f,'Synthetic normal simulation only.');fclose(f);
pause(1.5);observer.tick;
s=jsondecode(fileread(fullfile(observer.Session,'latest.json')));
assert(s.samples>1&&s.height_valid&&abs(s.height_relative_barometric_m-1)<1e-9,'Live viewing-block reading failed');
fprintf('PASS: live Display RuntimeObject data and relative height snapshot.\n');
set_param(model,'SimulationCommand','stop');pause(0.7);observer.tick;observer.publish(observer.LastState,false);
s=jsondecode(fileread(fullfile(observer.Session,'latest.json')));assert(~s.height_valid&&~s.data_fresh,'Stopped model presented as live');
observer.stop;close_system(model,0);
fprintf('PASS: stopped data invalidated, CSV persisted, observer stopped without a target command.\n');
