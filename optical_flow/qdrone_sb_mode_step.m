function [mode,diagnostic]=qdrone_sb_mode_step(axis,fresh,time,base,enabled)
%#codegen
persistent state
if isempty(state),state=zeros(6,1);end
if coder.target('MATLAB')
    [state,mode,diagnostic]=qdrone_sb_mode_mex(state,double(axis),double(fresh),double(time),double(base),double(enabled));
else
    [state,mode,diagnostic]=QDroneSBModeBuildable.step(state,double(axis),double(fresh),double(time),double(base),double(enabled));
end
end
