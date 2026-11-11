function test_yaw_i
% Exercise the actual MATLAB function. No hardware or Simulink needed.
clear qdrone_yaw_i_step
zero=zeros(3,1);
args={ [0;0;-1],1,10,0,zero,zero,0,1,0.01,0.015,0.003 };
previous=0;
for k=1:10000
    [u,on]=qdrone_yaw_i_step(args{:});
    assert(on==1 && all(u(1:2)==0));
    assert(u(3)<=0 && abs(u(3))<=0.015+1e-12);
    assert(abs(u(3)-previous)<=3e-6+1e-12,'Slew limit failed.');
    previous=u(3);
end
assert(abs(u(3)+0.015)<1e-10,'Magnitude limit or correction sign failed.');
args{1}=[0;0;1];args{7}=1;
[held,~]=qdrone_yaw_i_step(args{:});
assert(isequal(held,u),'Must freeze while allocation/torque is clipped.');
args{7}=0;
for k=1:10000
    [u,~]=qdrone_yaw_i_step(args{:});
    assert(abs(u(3)-previous)<=3e-6+1e-12);previous=u(3);
end
assert(u(3)>0,'Integral must unwind and reverse sign.');
% Each gate must clear a pre-existing integral, not just suppress its output.
for gate=1:10
    clear qdrone_yaw_i_step
    args={ [0;0;1],1,10,0,zero,zero,0,1,0.01,0.015,0.003 };
    for k=1:100,qdrone_yaw_i_step(args{:});end
    invalid=args;
    switch gate
        case 1,invalid{2}=0;                 % actual motor Arm gate
        case 2,invalid{3}=0;                 % low throttle
        case 3,invalid{4}=1;                 % direct torque mode
        case 4,invalid{5}=[16*pi/180;0;0];    % excessive tilt
        case 5,invalid{6}=[0;0;61*pi/180];    % excessive rotation
        case 6,invalid{8}=0;                 % explicit enable off
        case 7,invalid{1}=[0;0;NaN];
        case 8,invalid{9}=0.02;              % gain beyond trial range
        case 9,invalid{10}=0.1;              % torque beyond trial range
        case 10,invalid{11}=NaN;
    end
    [u,on]=qdrone_yaw_i_step(invalid{:});assert(on==0 && all(u==0));
    [u,on]=qdrone_yaw_i_step(args{:});
    assert(on==1 && u(3)>0 && u(3)<=3e-6+1e-12,'Gate did not reset state.');
end
clear qdrone_yaw_i_step
fprintf('PASS: yaw I sign, cap, slew, unwind, clipping freeze, reset gates, roll/pitch zero.\n');
end
