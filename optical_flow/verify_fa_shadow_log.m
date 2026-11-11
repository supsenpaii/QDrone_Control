function verification=verify_fa_shadow_log(logFile,intervals)
% intervals Nx3: [start_s end_s motion], motion +1 forward,-1 backward,
% +2 left,-2 right,0 rotation about CG with no intentional translation.
% Mark ONLY observed known motions of the DISARMED aircraft over textured floor.
% Verifies direction/rough gyro compensation, not flight stability or scale.
data=load(logFile,'flow_assist_data');assert(isfield(data,'flow_assist_data'),'Wrong log variable.');
a=data.flow_assist_data;assert(size(a,1)==66&&size(a,2)>50,'Expected finalized 66-row FA log.');
assert(all(isfinite(a(:))),'Non-finite log data.');
assert(all(diff(a(1,:))>0),'Log clock must increase.');
assert(all(a(24,:)<.5),'Verification log must be entirely Disarmed.');
assert(all(a(44,:)==2),'Log must use shadow stage=2 throughout.');
assert(max(max(abs(a(6:8,:)-a(3:5,:))))<1e-12,'Shadow changed pilot commands.');
cfg=a(44:56,1);R=reshape(a(57:65,1),3,3);
assert(max(max(abs(bsxfun(@minus,a(44:56,:),cfg))))<1e-12,'Configuration changed during recording.');
assert(max(max(abs(bsxfun(@minus,a(57:65,:),R(:)))))<1e-12,'Mounting matrix changed during recording.');
assert(isnumeric(intervals)&&size(intervals,2)==3&&size(intervals,1)>=5&&all(isfinite(intervals(:))),'Supply >=5 known-motion intervals.');
assert(all(ismember([-2 -1 0 1 2],intervals(:,3)')),'Need forward/backward/left/right and pure rotation segments.');
results=struct('time',{},'motion',{},'samples',{},'proxy_median',{},'angle_candidate_median',{});
for k=1:size(intervals,1)
    row=intervals(k,:);assert(row(2)-row(1)>=.5,'Each marked interval must span >=0.5s.');
    ix=a(1,:)>=row(1)&a(1,:)<=row(2)&a(28,:)==9;
    assert(nnz(ix)>=100,'Interval %d needs >=100 active shadow samples (status9).',k);
    v=median(a(36:37,ix),2);angles=median(a(38:39,ix),2);motion=row(3);
    if motion==0
        gyro=abs(a(18:20,ix));assert(max(median(gyro,2))>.1,'Rotation interval needs observable rotation.');
        assert(max(abs(v))<.08,'Rotation residual too large; inspect timing, intrinsics, height and mounting.');
    else
        axis=abs(motion);direction=sign(motion);
        assert(v(axis)*direction>.03,'Wrong/weak flow direction in interval %d.',k);
        assert(abs(v(3-axis))<2*abs(v(axis)),'Cross-axis response dominates interval %d.',k);
        if axis==1,assert(angles(2)*direction<0,'Pitch does not oppose X motion.');
        else,assert(angles(1)*direction>0,'Roll does not oppose Y motion.');end
    end
    results(k)=struct('time',row(1:2),'motion',motion,'samples',nnz(ix),'proxy_median',v,'angle_candidate_median',angles); %#ok<AGROW>
end
verification=struct('passed',true,'logFile',logFile,'config',cfg,'cameraToBody',R, ...
    'results',results,'flight_stability_validated',false,'ground_only',true);
save(fullfile(fileparts(mfilename('fullpath')),'fa_shadow_verification.mat'),'verification');
fprintf('PASS: known ground-motion directions, rotation residual and shadow passthrough. NOT flight stability validation.\n');
end
