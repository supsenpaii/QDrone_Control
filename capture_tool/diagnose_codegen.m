folder=fileparts(mfilename('fullpath'));addpath(folder,fullfile(fileparts(folder),'optical_flow'));cd(fileparts(folder));
cfg=coder.config('lib');cfg.GenCodeOnly=true;
try
 codegen('-config',cfg,'qdrone_capture_frame','-args',{zeros(480,640,'uint8'),false,0,1},'-d',fullfile(folder,'codegen_check','frame'));
 fprintf('PASS frame packet C generation\n');
 codegen('-config',cfg,'qdrone_capture_packet','-args',{0,zeros(5,1),zeros(11,1),zeros(8,1),0,0},'-d',fullfile(folder,'codegen_check','telemetry'));
 fprintf('PASS telemetry packet C generation\n');
catch cause
 fprintf('%s\n',getReport(cause,'extended','hyperlinks','off'));rethrow(cause);
end
