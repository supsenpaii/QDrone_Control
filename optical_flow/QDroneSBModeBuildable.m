classdef QDroneSBModeBuildable < coder.ExternalDependency %#codegen
    methods (Static)
        function name=getDescriptiveName(~),name='QDrone SB Rz mode selector';end
        function yes=isSupportedContext(~),yes=true;end
        function updateBuildInfo(info,~)
            folder=fileparts(mfilename('fullpath'));
            info.addIncludePaths({folder});info.addSourcePaths({folder});
            info.addSourceFiles({'qdrone_sb_mode_core.c'});info.addIncludeFiles({'qdrone_sb_mode_core.h'});
        end
        function [s,mode,d]=step(s,x,f,t,b,e)
            coder.inline('always');coder.cinclude('qdrone_sb_mode_core.h');
            mode=0;d=zeros(6,1);
            coder.ceval('qdrone_sb_mode',coder.ref(s),x,f,t,b,e,coder.wref(mode),coder.wref(d));
        end
    end
end
