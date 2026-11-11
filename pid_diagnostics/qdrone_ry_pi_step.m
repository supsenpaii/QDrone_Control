function [term,diagnostic]=qdrone_ry_pi_step(errorRate,angles,gyro,residual,arm,throttle,mode,enable,config,reset)
%#codegen
% Yaw-only integral. Legacy vector sizes preserve existing model/log callers.
persistent state
if isempty(state),state=zeros(4,1);end
er=reshape(errorRate,3,1);a=reshape(angles,3,1);g=reshape(gyro,3,1);
r=reshape(residual,3,1);en=reshape(enable,2,1);cfg=reshape(config,8,1);
en(1)=0;cfg(1)=0;state(1)=0;state(3)=0;
term=zeros(3,1);diagnostic=zeros(10,1);
if coder.target('MATLAB')
    [state,term,diagnostic]=qdrone_yaw_only_mex(state,er,a,g,r,arm,throttle,mode,en,cfg,reset,0.001);
else
    [state,term,diagnostic]=QDroneRYPIBuildable.step(state,er,a,g,r,arm,throttle,mode,en,cfg,reset);
end
term(1)=0;term(2)=0;
end
