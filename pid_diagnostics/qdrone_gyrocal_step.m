function [gyro, bias, ready, progress] = qdrone_gyrocal_step(raw, accel, arm)
%#codegen
% One 1 ms sample. Calibrate once per run, Disarmed, on a stationary rig.
persistent count sumG sumG2 sumA sumA2 estimate done
if isempty(count)
    count=0; sumG=zeros(3,1); sumG2=zeros(3,1);
    sumA=zeros(3,1); sumA2=zeros(3,1); estimate=zeros(3,1); done=false;
end
g=reshape(raw,3,1); a=reshape(accel,3,1);
if ~done
    valid = all(isfinite(g)) && all(isfinite(a)) && isfinite(arm) && arm<0.5;
    valid = valid && all(abs(g)<5*pi/180) && norm(a)>8 && norm(a)<11.5;
    if ~valid
        count=0; sumG(:)=0; sumG2(:)=0; sumA(:)=0; sumA2(:)=0;
    else
        count=count+1; sumG=sumG+g; sumG2=sumG2+g.*g;
        sumA=sumA+a; sumA2=sumA2+a.*a;
        if count>=5000
            meanG=sumG/count; meanA=sumA/count;
            varG=max(sumG2/count-meanG.*meanG,0);
            varA=max(sumA2/count-meanA.*meanA,0);
            if all(varG<(0.75*pi/180)^2) && all(varA<0.20^2)
                estimate=meanG; done=true;
            else
                count=0; sumG(:)=0; sumG2(:)=0; sumA(:)=0; sumA2(:)=0;
            end
        end
    end
end
bias=estimate;
gyro=g-estimate;
ready=double(done);
progress=min(count/5000,1);
end
