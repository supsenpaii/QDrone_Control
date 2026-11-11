function test_roll_i
clear qdrone_roll_i_step
e=[1;4;5];zero=zeros(3,1);
for k=1:5000
    [u,active]=qdrone_roll_i_step(e,1,10,1,0,zero,zero,0);
    assert(active==1 && abs(u(1))<=0.03+1e-12 && all(u(2:3)==0));
end
assert(abs(u(1)-0.03)<1e-10,'Integral magnitude limit failed.');
for k=1:100
    [held,~]=qdrone_roll_i_step(-e,1,10,1,0,zero,zero,1);
    assert(max(abs(held-u))<1e-12,'Must freeze during saturation.');
end
disabled={ {1,10,0,0,zero,zero,0}, {0,10,1,0,zero,zero,0}, ...
    {1,0,1,0,zero,zero,0}, {1,10,1,1,zero,zero,0}, ...
    {1,10,1,0,[11*pi/180;0;0],zero,0}, {1,10,1,0,zero,[31*pi/180;0;0],0} };
for j=1:numel(disabled)
    clear qdrone_roll_i_step
    qdrone_roll_i_step(e,1,10,1,0,zero,zero,0);
    inputs=disabled{j};
    [u,active]=qdrone_roll_i_step(e,inputs{:});
    assert(active==0 && all(u==0),'Inactive conditions must reset integral.');
end
clear qdrone_roll_i_step
[u,~]=qdrone_roll_i_step(e,1,10,1,0,zero,zero,0);
assert(u(1)>0 && u(1)<=1e-5+1e-12,'Wrong correction sign or slew limit.');
[u,active]=qdrone_roll_i_step([NaN;0;0],1,10,1,0,zero,zero,0);
assert(active==0 && all(u==0),'Invalid data must reset integral.');
clear qdrone_roll_i_step
fprintf('PASS: roll I sign, magnitude/slew limits, saturation freeze, reset gates, pitch/yaw zero.\n');
end
