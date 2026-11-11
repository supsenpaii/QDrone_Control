function build_mex
% Compile all current C gateways from source using the configured MEX compiler.
% Requires MATLAB Coder for the wrappers; does not access drone hardware.
project=setup_project;
clear qdrone_fa_step qdrone_sb_mode_step qdrone_ry_pi_step
clear qdrone_fa_mex qdrone_fa_timed_mex qdrone_sb_mode_mex qdrone_yaw_only_mex
flow=fullfile(project,'optical_flow');pid=fullfile(project,'pid_diagnostics');
for name={'qdrone_fa_mex','qdrone_fa_timed_mex'}
    mex('-outdir',flow,'-output',name{1}, ...
        fullfile(flow,'qdrone_fa_mex.c'),fullfile(flow,'qdrone_fa_core.c'));
end
mex('-outdir',flow,'-output','qdrone_sb_mode_mex', ...
    fullfile(flow,'qdrone_sb_mode_mex.c'),fullfile(flow,'qdrone_sb_mode_core.c'));
mex('-outdir',pid,'-output','qdrone_yaw_only_mex', ...
    fullfile(pid,'qdrone_ry_pi_mex.c'),fullfile(pid,'qdrone_ry_pi_core.c'));
fprintf('Built current Flow Assist, SB and yaw-only MEX gateways from C source.\n');
end
