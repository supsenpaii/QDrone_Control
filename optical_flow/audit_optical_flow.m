project=fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(project,'optical_flow'));
cd(project);
fprintf('=== ALGORITHM TESTS ===\n');
test_qdrone_of;
fprintf('=== CODE ANALYZER ===\n');
disp(checkcode(fullfile(project,'optical_flow','qdrone_of_step.m'),'-id'));
fprintf('=== RECORDED DATA ===\n');
files=dir(fullfile(project,'saved_data','Stabilizer_QDrone_048378_OF','OpticalFlow_*.mat'));
for k=1:numel(files)
    if files(k).bytes==0, continue; end
    file=fullfile(files(k).folder,files(k).name);
    fprintf('FILE %s\n',file);
    try
        data=load(file);
        names=fieldnames(data);
        for j=1:numel(names)
            v=data.(names{j});
            fprintf('VARIABLE %s class=%s size=%s\n',names{j},class(v),mat2str(size(v)));
            if isnumeric(v)
                disp(v(:,1:min(5,size(v,2))));
            end
        end
    catch cause
        fprintf('LOG READ ERROR: %s\n',cause.message);
    end
end
fprintf('=== MODEL STRUCTURE ===\n');
model='Stabilizer_QDrone_048378_OF';
load_system(fullfile(project,[model '.slx']));
monitor=[model '/STABILIZER - QDRONE/Optical Flow Monitor'];
blocks=find_system(monitor,'SearchDepth',1,'Type','Block');
for k=1:numel(blocks)
    fprintf('BLOCK %s\n',blocks{k});
    params=get_param(blocks{k},'DialogParameters');
    keys={'sample_time','SampleTime','sampleTime','OutPortSampleTime','Integrity','Deterministic','Inputs'};
    for j=1:numel(keys)
        if isstruct(params) && isfield(params,keys{j})
            fprintf('  %s=%s\n',keys{j},get_param(blocks{k},keys{j}));
        end
    end
end
set_param(model,'SimulationCommand','update');
fprintf('PASS: diagram update. No model started or hardware armed.\n');
close_system(model,0);
