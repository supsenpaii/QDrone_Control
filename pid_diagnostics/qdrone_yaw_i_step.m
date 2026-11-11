function [term,active]=qdrone_yaw_i_step(errorRate,arm,throttle,mode,angles,gyro,limited,enable,ki,cap,slew)
%#codegen
% Bounded yaw-rate integral EXPERIMENT, fixed 1 ms update. Not heading hold.
% Limits are conservative trial bounds, not flight-validated tuning.
persistent integral
if isempty(integral),integral=0;end
er=reshape(errorRate,3,1);a=reshape(angles,3,1);g=reshape(gyro,3,1);
finite=all(isfinite(er)) && all(isfinite(a)) && all(isfinite(g)) && ...
    all(isfinite([arm throttle mode limited enable ki cap slew]));
parameters=finite && ki>0 && ki<=0.01 && cap>0 && cap<=0.015 && slew>0 && slew<=0.003;
enabled=parameters && enable>0.5 && arm>0.5 && throttle>=4 && mode<0.5 && ...
    all(abs(a(1:2))<15*pi/180) && all(abs(g)<60*pi/180);
if ~enabled
    integral=0;
else
    % Apply a reduced cap even when saturation freezes further integration.
    integral=min(max(integral,-cap),cap);
    if limited<0.5
        delta=ki*er(3)*0.001;
        delta=min(max(delta,-slew*0.001),slew*0.001);
        integral=min(max(integral+delta,-cap),cap);
    end
end
term=[0;0;integral];active=double(enabled);
end
