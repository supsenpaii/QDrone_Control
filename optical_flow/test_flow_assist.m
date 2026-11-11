function test_flow_assist
% Synthetic checks through the shared C/MEX implementation. No hardware access.
assert(isfile(fullfile(fileparts(mfilename('fullpath')),'qdrone_fa_mex.mexw64')), ...
    'Run optical_flow/build_fa_native.cmd first.');
assert(isfile(fullfile(fileparts(mfilename('fullpath')),'qdrone_fa_timed_mex.mexw64')), ...
    'Run optical_flow/build_fa_native.cmd to build the timestamped-gyro wrapper.');
cfg=[1;400;400;1;.03;3*pi/180;4*pi/180;.1;.35;.1;.35;pi/180;12*pi/180];
R=[0 -1 0;-1 0 0;0 0 -1];cmd=[0;0;.2];g=zeros(3,1);a=g;
flow=[0;.4*400*.033;.8;1;.033;0;30;28];state=zeros(16,1);ft=0;
for k=1:1200
    t=k*.001;if mod(k,33)==0,ft=t;end
    [state,out,d]=qdrone_fa_mex(state,cmd,flow,ft,g,a,1,10,0,1,cfg,R,t);
end
assert(d(1)==5&&out(2)<-.01&&abs(out(1))<1e-12&&out(3)==cmd(3));
[state,out,d]=qdrone_fa_mex(state,cmd,flow,ft,g,a,0,10,0,1,cfg,R,1.201); %#ok<ASGLU>
assert(isequal(out,cmd),'Disarm must restore the exact pilot command.');
cfg(1)=0;[~,out,d]=qdrone_fa_mex(state,cmd,flow,ft,g,a,1,10,0,1,cfg,R,1.202);
assert(isequal(out,cmd)&&d(1)==1);
clear qdrone_fa_step
cfg(1)=1;ft=0;
for k=1:1200
    t=k*.001;if mod(k,33)==0,ft=t;end
    [out,d]=qdrone_fa_step(cmd,flow,ft,g,a,1,10,0,1,cfg,R,t);
end
assert(d(1)==5&&out(2)<-.01&&out(3)==cmd(3));
clear qdrone_fa_step
fprintf('PASS: Flow Assist MATLAB wrapper/shared MEX: damping sign, yaw passthrough, Disarm, calibration lock.\n');
end
