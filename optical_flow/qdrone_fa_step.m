function [output,diagnostic]=qdrone_fa_step(command,flow,frameTime,gyro,attitude,arm,throttle,mode,request,config,rotation,time)
%#codegen
persistent state
if isempty(state),state=zeros(1045,1);end
cmd=reshape(command,3,1);f=reshape(flow,8,1);g=reshape(gyro,3,1);a=reshape(attitude,3,1);
c=reshape(config,13,1);R=reshape(rotation,3,3);
if coder.target('MATLAB')
    [state,output,diagnostic]=qdrone_fa_timed_mex(state,cmd,f,frameTime,g,a,arm,throttle,mode,request,c,R,time);
else
    [state,output,diagnostic]=QDroneFlowAssistBuildable.step(state,cmd,f,frameTime,g,a,arm,throttle,mode,request,c,R,time);
end
end
