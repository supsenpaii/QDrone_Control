function test_roll_yaw_controller
% Test the actual MEX/C core through the MATLAB wrapper. No hardware access.
folder=fileparts(mfilename('fullpath'));addpath(folder);
if ~isfile(fullfile(folder,['qdrone_yaw_only_mex.' mexext]))
    mex('-outdir',folder,'-output','qdrone_yaw_only_mex', ...
        fullfile(folder,'qdrone_ry_pi_mex.c'),fullfile(folder,'qdrone_ry_pi_core.c'));
end
clear qdrone_ry_pi_step
z=zeros(3,1);cfg=[.02;.01;.10;.015;.01;.003;1;1];en=ones(2,1);
for k=1:10000
    [u,d]=qdrone_ry_pi_step([1;0;-1],z,z,z,1,10,0,en,cfg,0);
    assert(u(1)==0 && u(2)==0 && abs(u(3))<=.015+1e-12);
end
assert(u(1)==0 && abs(u(3)+.015)<1e-10);
held=u;
for k=1:100
    [u,d]=qdrone_ry_pi_step([1;0;-1],z,[0;70*pi/180;0],z,1,10,0,en,cfg,0);
    assert(isequal(u,held) && d(1)==0 && d(2)==2,'Yaw transient hold failed.');
end
for k=1:4000
    [u,d]=qdrone_ry_pi_step([1;0;-1],z,[0;70*pi/180;0],z,1,10,0,en,cfg,0);
end
assert(all(abs(u)<1e-12) && d(1)==0 && d(2)==4,'Sustained excursion must shed yaw I.');
clear qdrone_ry_pi_step
for k=1:1000,[u,~]=qdrone_ry_pi_step([1;0;-1],z,z,z,1,10,0,en,cfg,0);end
before=u;
[u,d]=qdrone_ry_pi_step([1;0;-1],z,z,[-.01;0;.01],1,10,0,en,cfg,0);
assert(u(1)==0 && u(3)>before(3) && d(5)==0 && d(6)==1,'Yaw anti-windup sign failed.');
[u,~]=qdrone_ry_pi_step([1;0;-1],z,z,z,0,10,0,en,cfg,0);assert(all(u==0));
[u,~]=qdrone_ry_pi_step([1;0;-1],z,z,z,1,10,0,[1;0],cfg,0);
assert(all(u==0),'Legacy roll enable must produce no integral torque.');
[u,~]=qdrone_ry_pi_step([1;0;-1],z,z,z,1,10,0,en,cfg,1);assert(all(u==0));
clear qdrone_ry_pi_step
fprintf('PASS: yaw-only wrapper/MEX, zero roll torque, cap/slew, hold/decay, anti-windup and reset.\n');
end
