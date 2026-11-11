folder=fileparts(mfilename('fullpath'));addpath(folder);cd(fileparts(folder));
test_baro_height;
cfg=coder.config('lib');cfg.GenCodeOnly=true;
codegen('-config',cfg,'qdrone_baro_height','-args',{0,0,0,0,false},'-d',fullfile(folder,'codegen_check','height'));
fprintf('PASS: height estimator C generation.\n');
install_baro_height;
