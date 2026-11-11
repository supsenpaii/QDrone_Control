function output=save_fa_calibration_image(recorder)
% Save the latest recorded raw down-camera frame with detected checkerboard.
% Call once per stationary board pose while QDrone_Capture records images.
folder=fileparts(mfilename('fullpath'));
addpath(fullfile(fileparts(folder),'capture_tool'));
assert(isa(recorder,'QDroneCapture') && isvalid(recorder),'Supply the recorder returned by open_capture_tool.');
assert(~recorder.Closing && recorder.ReceiveFrames,'Start the capture receiver with image recording enabled.');
assert(exist('detectCheckerboardPoints','file')~=0,'Computer Vision Toolbox checkerboard detection is required.');
assert(recorder.CameraLast(1)>=0 && toc(recorder.StartClock)-recorder.CameraLast(1)<3, ...
    'No recent down-camera frame. Check the capture connection.');
session=recorder.Session;
metadata=readtable(fullfile(session,'frames.csv'));
rows=find(metadata.camera_id==1);
assert(~isempty(rows),'No down-camera frames recorded yet.');
row=rows(end);
image=read_qdrone_frame(session,1,numel(rows));
assert(isa(image,'uint8') && isequal(size(image),[480 640]),'Expected raw 640x480 grayscale down-camera image.');
[points,boardSize]=detectCheckerboardPoints(image);
assert(~isempty(points),'Checkerboard not detected. Adjust lighting, distance or board pose and try again.');
destination=fullfile(session,'calibration_images');
if ~isfolder(destination),mkdir(destination);end
output=fullfile(destination,sprintf('down_epoch_%d_frame_%.0f.png',metadata.epoch(row),metadata.frame_seq(row)));
assert(~isfile(output),'This frame was saved already. Wait for a fresh frame and change the board pose.');
imwrite(image,output);
fprintf('Saved %s\nTarget time %.3f s, board %d x %d squares.\n', ...
    output,metadata.target_s(row),boardSize(1),boardSize(2));
end
