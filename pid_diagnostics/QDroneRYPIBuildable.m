classdef QDroneRYPIBuildable < coder.ExternalDependency %#codegen
    % Register C files in a host-side build hook, not in generated chart code.
    methods (Static)
        function name=getDescriptiveName(~)
            name='QDrone yaw-only integral core';
        end
        function yes=isSupportedContext(~)
            yes=true; % Pure portable C: host simulation and QUARC Linux x64.
        end
        function updateBuildInfo(buildInfo,~)
            folder=fileparts(mfilename('fullpath'));
            buildInfo.addIncludePaths({folder});
            buildInfo.addSourcePaths({folder});
            buildInfo.addSourceFiles({'qdrone_ry_pi_core.c'});
            buildInfo.addIncludeFiles({'qdrone_ry_pi_core.h'});
        end
        function [state,term,diagnostic]=step(state,er,a,g,r,arm,throttle,mode,en,cfg,reset)
            coder.inline('always');coder.cinclude('qdrone_ry_pi_core.h');
            term=zeros(3,1);diagnostic=zeros(10,1);
            coder.ceval('qdrone_ry_core_step',coder.ref(state),coder.rref(er),coder.rref(a), ...
                coder.rref(g),coder.rref(r),arm,throttle,mode,coder.rref(en),coder.rref(cfg), ...
                reset,0.001,coder.wref(term),coder.wref(diagnostic));
        end
        function residual=allocator(m,raw,lim,inv)
            coder.inline('always');coder.cinclude('qdrone_ry_pi_core.h');
            residual=zeros(3,1);
            coder.ceval('qdrone_ry_core_aw',coder.rref(m),coder.rref(raw),coder.rref(lim), ...
                coder.rref(inv),coder.wref(residual));
        end
    end
end
