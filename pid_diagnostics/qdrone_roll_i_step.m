function [term,active] = qdrone_roll_i_step(errorRate,arm,throttle,ready,mode,angles,gyro,limited)
%#codegen
% Small roll-only integral trial, fixed sample time 0.001 s.
persistent integral
if isempty(integral), integral=0; end
er=reshape(errorRate,3,1); a=reshape(angles,3,1); g=reshape(gyro,3,1);
valid=all(isfinite(er)) && all(isfinite(a)) && all(isfinite(g)) && ...
    isfinite(arm) && isfinite(throttle) && isfinite(ready) && isfinite(mode) && isfinite(limited);
enabled=valid && arm>0.5 && ready>0.5 && mode<0.5 && throttle>=4 && ...
    all(abs(a(1:2))<10*pi/180) && abs(g(1))<30*pi/180;
if ~enabled
    integral=0;
elseif limited<0.5
    % Ki=0.02 Nm/rad; cap change at 0.01 Nm/s and magnitude at 0.03 Nm.
    delta=0.02*er(1)*0.001;
    delta=min(max(delta,-0.01*0.001),0.01*0.001);
    integral=min(max(integral+delta,-0.03),0.03);
end
term=[integral;0;0];
active=double(enabled);
end
