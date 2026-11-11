function test_gyrocal
% Exercise calibration, early Arm, disturbance rejection and frozen bias.
raw=[-1.3;0.1;-0.2]*pi/180; accel=[0;0;9.81];
clear qdrone_gyrocal_step
for k=1:4999
    [~,~,ready]=qdrone_gyrocal_step(raw,accel,0);
    assert(ready==0,'Calibration completed too early.');
end
[gyro,bias,ready]=qdrone_gyrocal_step(raw,accel,0);
assert(ready==1 && max(abs(bias-raw))<1e-10 && max(abs(gyro))<1e-10);
for k=1:100
    [gyro,newBias,ready]=qdrone_gyrocal_step(raw+[0.2;0;0],accel,1);
    assert(ready==1 && max(abs(newBias-bias))<1e-12 && abs(gyro(1)-0.2)<1e-10);
end
clear qdrone_gyrocal_step
for k=1:6000
    [~,~,ready,progress]=qdrone_gyrocal_step(raw,accel,1);
    assert(ready==0 && progress==0,'Early Arm must prevent calibration.');
end
for k=1:1000, qdrone_gyrocal_step(raw,accel,0); end
[~,~,ready,progress]=qdrone_gyrocal_step([NaN;0;0],accel,0);
assert(ready==0 && progress==0,'Invalid data must reset calibration.');
for k=1:5000
    noisy=raw; noisy(1)=noisy(1)+(-1)^k*1*pi/180;
    [~,~,ready]=qdrone_gyrocal_step(noisy,accel,0);
end
assert(ready==0,'Excess gyro variance must reject calibration.');
for k=1:5000, [~,~,ready]=qdrone_gyrocal_step(raw,accel,0); end
assert(ready==1,'Stable data should recover after rejection.');
clear qdrone_gyrocal_step
fprintf('PASS: stationary calibration, early Arm lockout, invalid/noisy rejection, frozen bias.\n');
end
