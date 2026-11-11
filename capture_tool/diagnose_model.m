folder=fileparts(mfilename('fullpath'));addpath(folder,fullfile(fileparts(folder),'optical_flow'));cd(fileparts(folder));
try
 build_qdrone_capture(true);
 fprintf('PASS: complete sensor-only model with all five camera streams.\n');
catch cause
 fprintf('%s\n',getReport(cause,'extended','hyperlinks','off'));
 try,disp(sllasterror);catch,end
 rethrow(cause);
end
