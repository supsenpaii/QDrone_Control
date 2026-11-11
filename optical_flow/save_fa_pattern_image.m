function output=save_fa_pattern_image(recorder,label)
% Save a raw frame for the measured-translation pattern experiment.
assert(any(strcmp(label,{'reference','forward','left'})),'Use reference, forward or left.');
folder=fileparts(mfilename('fullpath'));
addpath(fullfile(fileparts(folder),'capture_tool'));
assert(isa(recorder,'QDroneCapture') && isvalid(recorder),'Supply the active Capture recorder.');
assert(~recorder.Closing && recorder.ReceiveFrames,'Image recorder is stopped.');
assert(recorder.CameraLast(1)>=0 && toc(recorder.StartClock)-recorder.CameraLast(1)<3, ...
    'No recent down-camera image.');
metadata=readtable(fullfile(recorder.Session,'frames.csv'));
rows=find(metadata.camera_id==1);assert(~isempty(rows),'No down-camera frames.');
image=read_qdrone_frame(recorder.Session,1,numel(rows));
assert(isa(image,'uint8') && isequal(size(image),[480 640]),'Expected raw 640x480 grayscale image.');
destination=fullfile(recorder.Session,'pattern_calibration');
if ~isfolder(destination),mkdir(destination);end
output=fullfile(destination,[label '.png']);
assert(~isfile(output),'This pose already exists. Start a new Capture session for a fresh experiment.');
frameMetadata=metadata(rows(end),:);
imwrite(image,output);
save(fullfile(destination,[label '.mat']),'frameMetadata');
fprintf('Saved %s at target time %.3f s.\n',output,frameMetadata.target_s);
end
