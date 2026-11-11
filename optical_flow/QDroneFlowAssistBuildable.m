classdef QDroneFlowAssistBuildable < coder.ExternalDependency %#codegen
    methods (Static)
        function name=getDescriptiveName(~),name='QDrone image-space Flow Assist';end
        function yes=isSupportedContext(~),yes=true;end
        function updateBuildInfo(info,~)
            folder=fileparts(mfilename('fullpath'));
            info.addIncludePaths({folder});info.addSourcePaths({folder});
            info.addSourceFiles({'qdrone_fa_core.c'});info.addIncludeFiles({'qdrone_fa_core.h'});
        end
        function [s,out,d]=step(s,cmd,f,ft,g,a,arm,throttle,mode,req,c,R,t)
            coder.inline('always');coder.cinclude('qdrone_fa_core.h');
            out=zeros(3,1);d=zeros(16,1);
            coder.ceval('qdrone_fa_step_timed',coder.ref(s),coder.rref(cmd),coder.rref(f),ft, ...
                coder.rref(g),coder.rref(a),arm,throttle,mode,req,coder.rref(c),coder.rref(R),t, ...
                coder.wref(out),coder.wref(d));
        end
    end
end
