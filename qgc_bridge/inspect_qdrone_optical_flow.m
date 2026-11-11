function report=inspect_qdrone_optical_flow(ip)
% Read-only diagnosis: never starts/stops/downloads a model or opens hardware.
if nargin<1,ip='192.168.2.52';end
folder=fileparts(mfilename('fullpath'));project=fileparts(folder);
report=struct('checked_at',datestr(now,31),'ip',ip,'target_models',{{}}, ...
    'target_query_error','','open_models',{{}},'historical_flow',struct());
fprintf('=== READ ONLY: QDrone optical flow ===\n');
try
    [models,err]=qc_get_loaded_models(sprintf('tcpip://%s:17000',ip));
    fprintf('QUARC target result=%d; loaded models:\n',err);disp(models);
    report.target_models=models;
    if err<0,report.target_query_error=sprintf('QUARC error %d',err);end
catch cause
    report.target_query_error=cause.message;fprintf('TARGET QUERY FAILED: %s\n',cause.message);
end
loaded=find_system('SearchDepth',0,'Type','block_diagram');
for k=1:numel(loaded)
    model=loaded{k};
    if isempty(strfind(model,'QDrone')),continue;end %#ok<STREMP>
    blocks=find_system(model,'LookUnderMasks','all','FollowLinks','off','Type','Block');
    flowBlocks={};cameraBlocks={};
    for j=1:numel(blocks)
        name=get_param(blocks{j},'Name');
        if ~isempty(regexpi(name,'optical.*flow|flow status','once')),flowBlocks{end+1}=blocks{j};end %#ok<AGROW>
        params=get_param(blocks{j},'DialogParameters');
        if isstruct(params) && isfield(params,'device')
            try
                source=get_param(blocks{j},'ReferenceBlock');
                if ~isempty(strfind(source,'Video Capture')) %#ok<STREMP>
                    cameraBlocks{end+1}=struct('block',blocks{j}, ...
                        'device',get_param(blocks{j},'device'), ...
                        'sample_time',get_param(blocks{j},'sample_time')); %#ok<AGROW>
                end
            catch
            end
        end
    end
    entry=struct('name',model,'simulation_status',get_param(model,'SimulationStatus'), ...
        'simulation_mode',get_param(model,'SimulationMode'), ...
        'flow_blocks',{flowBlocks},'camera_blocks',{cameraBlocks});
    report.open_models{end+1}=entry;
    fprintf('MODEL %s: status=%s, mode=%s, flow blocks=%d, camera blocks=%d\n', ...
        model,entry.simulation_status,entry.simulation_mode,numel(flowBlocks),numel(cameraBlocks));
    disp(flowBlocks);
end
files=dir(fullfile(project,'saved_data','**','OpticalFlow_*.mat'));
[~,order]=sort([files.datenum],'descend');
for index=order
    file=fullfile(files(index).folder,files(index).name);
    try
        data=load(file,'optical_flow_data');
        if ~isfield(data,'optical_flow_data'),continue;end
        v=data.optical_flow_data;
        if size(v,1)~=16 || size(v,2)<2,continue;end
        good=isfinite(v(6,:)) & v(6,:)>0.5;
        summary=struct('file',file,'file_modified',datestr(files(index).datenum,31), ...
            'samples',size(v,2),'valid_samples',sum(good), ...
            'valid_fraction',mean(good),'duration_s',v(1,end)-v(1,1), ...
            'historical_not_live',true);
        report.historical_flow=summary;
        fprintf('HISTORICAL LOG (not live): %s\n',file);
        fprintf('samples=%d valid=%d (%.1f%%) duration=%.3f s\n', ...
            summary.samples,summary.valid_samples,100*summary.valid_fraction,summary.duration_s);
        break;
    catch
    end
end
fprintf('No model changed. No camera/HIL opened. No motor command sent.\n');
path=fullfile(folder,'optical_flow_diagnostic.json');
f=fopen(path,'w');
if f~=-1,fwrite(f,jsonencode(report),'char');fclose(f);fprintf('Report: %s\n',path);end
end
