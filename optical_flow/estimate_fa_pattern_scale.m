function report=estimate_fa_pattern_scale(folder,height_m,distance_m)
% Experimental focal scale from known camera translation over a flat pattern.
% Camera must stay level at the measured optical depth, with no rotation.
validateattributes(height_m,{'numeric'},{'real','finite','scalar','>=',0.3,'<=',3});
validateattributes(distance_m,{'numeric'},{'real','finite','positive','scalar'});
reference=imread(fullfile(folder,'reference.png'));
labels={'forward','left'};
focal=nan(2,1);axesUsed=zeros(2,1);results=cell(2,1);
for k=1:2
    image=imread(fullfile(folder,[labels{k} '.png']));
    assert(isequal(size(reference),[480 640]) && isequal(size(image),size(reference)), ...
        'Expected raw 640x480 grayscale frames.');
    [features0,points0]=extractFeatures(reference,detectSURFFeatures(reference));
    [features1,points1]=extractFeatures(image,detectSURFFeatures(image));
    pairs=matchFeatures(features0,features1,'Unique',true);
    assert(size(pairs,1)>=30,'Too few matched points for %s. Reduce motion or improve image sharpness.',labels{k});
    [transform,inliers0,inliers1]=estimateGeometricTransform( ...
        points0(pairs(:,1)),points1(pairs(:,2)),'similarity','MaxDistance',2);
    assert(inliers0.Count>=30 && inliers0.Count/size(pairs,1)>=0.5, ...
        'Insufficient consistent matches for %s.',labels{k});
    matrix=transform.T;
    rotation_deg=atan2(matrix(1,2),matrix(1,1))*180/pi;
    scale=hypot(matrix(1,1),matrix(1,2));
    assert(abs(rotation_deg)<1 && abs(scale-1)<0.03, ...
        'Motion %s: rotation %.2f deg (limit 1), scale %.4f (allowed 0.97-1.03). Keep orientation and height fixed.', ...
        labels{k},rotation_deg,scale);
    displacement=median(inliers1.Location-inliers0.Location,1);
    [magnitude,axis]=max(abs(displacement));
    assert(magnitude>=15 && abs(displacement(3-axis))<0.15*magnitude, ...
        'Motion for %s is too small or not aligned with an image axis.',labels{k});
    axesUsed(k)=axis;focal(axis)=height_m*magnitude/distance_m;
    results{k}=struct('motion',labels{k},'displacement_px',displacement, ...
        'inliers',inliers0.Count,'rotation_deg',rotation_deg,'scale',scale);
end
assert(axesUsed(1)~=axesUsed(2),'Forward and left must produce different image axes.');
report=struct('method','measured planar translation, experimental', ...
    'height_m',height_m,'distance_m',distance_m,'fx_fy_px',focal, ...
    'measurements',{results},'distortion_estimated',false,'flight_validated',false);
save(fullfile(folder,'pattern_scale_estimate.mat'),'report');
disp(report);disp(results{1});disp(results{2});
fprintf('Experimental fx %.2f px, fy %.2f px. Review matches and repeat before configuring FA.\n',focal);
figure;showMatchedFeatures(reference,image,inliers0,inliers1);
title('Left motion matched points');
end
