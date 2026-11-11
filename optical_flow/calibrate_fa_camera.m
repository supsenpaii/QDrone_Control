function cameraParams=calibrate_fa_camera(imageFolder,squareSize_mm)
% Offline calibration from actual 640x480 down-camera checkerboard images.
% No camera/HIL/network opened; no controller parameters changed.
validateattributes(squareSize_mm,{'numeric'},{'real','finite','positive','scalar'});
assert(isfolder(imageFolder),'Image folder missing.');
files={};
for pattern={'*.png','*.jpg','*.jpeg','*.bmp','*.tif','*.tiff'}
    found=dir(fullfile(imageFolder,pattern{1}));
    for k=1:numel(found),files{end+1}=fullfile(found(k).folder,found(k).name);end %#ok<AGROW>
end
files=unique(files);assert(numel(files)>=15,'Use at least 15 distinct checkerboard images with diverse poses.');
for k=1:numel(files)
    meta=imfinfo(files{k});assert(meta(1).Height==480&&meta(1).Width==640,'Image must be 640x480: %s',files{k});
end
[points,boardSize,used]=detectCheckerboardPoints(files);
assert(nnz(used)>=15,'Checkerboard detection requires >=15 usable images; found %d.',nnz(used));
world=generateCheckerboardPoints(boardSize,squareSize_mm);
[cameraParams,estimated,errors]=estimateCameraParameters(points,world,'ImageSize',[480 640], ...
    'EstimateSkew',false,'EstimateTangentialDistortion',true,'NumRadialDistortionCoefficients',2);
assert(nnz(estimated)>=15,'Too few views retained by calibration.');
output=fullfile(fileparts(mfilename('fullpath')),'fa_camera_params.mat');
if isfile(output),copyfile(output,[output '.' datestr(now,'yyyymmdd_HHMMSS') '.bak']);end
save(output,'cameraParams','files','used','estimated','errors','squareSize_mm');
fprintf('Saved %s; fx=%.3f, fy=%.3f px; mean reprojection error=%.3f px.\n',output,cameraParams.FocalLength,cameraParams.MeanReprojectionError);
fprintf('Inspect reprojection errors/coverage in Camera Calibrator before use; a small average alone does not validate the model.\n');
end
