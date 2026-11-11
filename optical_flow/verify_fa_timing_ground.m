function verification=verify_fa_timing_ground(directionFile,intervals,rotationFile,rotationIntervals)
% Ground-only evidence. Intervals must describe motions observed by the operator.
% Directions: +1 forward, -1 backward, +2 left, -2 right. Rotation windows are
% known stationary-position rotations with small incidental roll/pitch allowed.
direction=readLog(directionFile);rotation=readLog(rotationFile);
assert(isequal(direction(44:65,1),rotation(44:65,1)), ...
    'Ground logs must use the same stage, camera and controller configuration.');
assert(size(intervals,2)==3&&all(isfinite(intervals(:))) ...
    &&all(ismember([-2 -1 1 2],intervals(:,3)')),'Supply all four observed directions.');
for k=1:size(intervals,1)
    row=intervals(k,:);ix=window(direction,row(1:2));
    v=median(direction(36:37,ix),2);angle=median(direction(38:39,ix),2);
    axis=abs(row(3));sgn=sign(row(3));
    assert(ismember(axis,[1 2])&&v(axis)*sgn>.03,'Wrong or weak observed direction.');
    assert(abs(v(3-axis))<2*abs(v(axis)),'Cross-axis motion dominates.');
    if axis==1,assert(angle(2)*sgn<0,'Pitch sign failed.');
    else,assert(angle(1)*sgn>0,'Roll sign failed.');end
end
assert(size(rotationIntervals,2)==2&&size(rotationIntervals,1)>=2 ...
    &&all(isfinite(rotationIntervals(:))),'Supply at least two observed rotation windows.');
quiet=rotation(28,:)==9&all(abs(rotation(18:20,:))<.015,1);
assert(nnz(quiet)>=500,'Insufficient quiet samples for gyro bias/noise estimate.');
bias=median(rotation(18:20,quiet),2);noise=std(rotation(20,quiet),1);
angles=zeros(size(rotationIntervals,1),1);residual=zeros(size(angles));
for k=1:numel(angles)
    ix=window(rotation,rotationIntervals(k,:));time=rotation(1,ix);
    delta=trapz(time,bsxfun(@minus,rotation(18:20,ix),bias),2);
    angles(k)=delta(3);
    assert(abs(delta(3))>=.7*pi/180&&abs(delta(3))>1.5*max(abs(delta(1:2))) ...
        &&abs(delta(3))/(time(end)-time(1))>2*noise, ...
        'Yaw must be distinguishable from quiet noise and incidental roll/pitch.');
    sorted=sort(abs(rotation(36:37,ix)),2);
    residual(k)=max(sorted(:,ceil(.95*size(sorted,2))));
    assert(residual(k)<.08,'Rotation flow residual is too large.');
end
assert(any(angles>0)&&any(angles<0),'Need both observed yaw directions.');
verification=struct('passed',true,'ground_only',true,'small_rotation_only',true, ...
    'flight_stability_validated',false,'config',direction(44:56,1), ...
    'cameraToBody',reshape(direction(57:65,1),3,3),'image_delay_s',.034, ...
    'directionFile',directionFile,'directionIntervals',intervals, ...
    'rotationFile',rotationFile,'rotationIntervals',rotationIntervals, ...
    'yaw_angles_rad',angles,'rotation_residual_p95',residual);
save(fullfile(fileparts(mfilename('fullpath')),'fa_timing_ground_verification.mat'),'verification');
fprintf('PASS: observed directions, small yaw rotations, shadow passthrough and matching configuration.\n');
end

function a=readLog(path)
data=load(path,'flow_assist_data');a=data.flow_assist_data;
assert(size(a,1)==66&&size(a,2)>500&&all(isfinite(a(:))) ...
    &&all(diff(a(1,:))>0),'Invalid or incomplete FA log.');
assert(all(a(24,:)<.5)&&all(a(44,:)==3),'Require entirely Disarmed stage 3 logs.');
assert(max(max(abs(a(6:8,:)-a(3:5,:))))<1e-12,'Shadow changed commands.');
assert(all(all(bsxfun(@eq,a(44:65,:),a(44:65,1)))),'Configuration changed in log.');
end

function ix=window(a,times)
assert(times(2)-times(1)>=.5,'Each window must span at least .5 s.');
allSamples=a(1,:)>=times(1)&a(1,:)<=times(2);
ix=allSamples&a(28,:)==9;
assert(nnz(ix)>=100&&nnz(ix)>=.95*nnz(allSamples), ...
    'Window needs at least 100 shadow samples and 95 percent active coverage.');
end
