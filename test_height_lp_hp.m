function test_height_lp_hp
% Synthetic tests only; no hardware or model is started.
clear qdrone_height_lp_hp_step;
base = [0;0;0;1;101325;1;0.3274;1];
dt=0.025;
y=qdrone_height_lp_hp_step(base,0,false);
assert(numel(y)==10 && y(1)==0 && y(9)==0);
% High-frequency stationary disturbance: +/-40 cm at 0.5 Hz.
samples=zeros(800,1);
for k=1:1600
    h=0.4*sin(2*pi*0.5*k*dt);
    base(1)=h; base(2)=h;
    y=qdrone_height_lp_hp_step(base,k*dt,false);
    if k>800, samples(k-800)=y(1); end
end
assert(max(abs(samples))<0.2,'LP failed the synthetic 20 cm target');
% A held nonzero height must remain nonzero; HP must decay to zero.
base(1)=1; base(2)=1;
for k=1601:4000
    y=qdrone_height_lp_hp_step(base,k*dt,false);
end
assert(abs(y(1)-1)<1e-6 && abs(y(9))<1e-6,'DC behavior incorrect');
assert(y(10)==1,'Kalman diagnostic changed');
base(4)=0; base(1)=NaN; base(2)=NaN;
y=qdrone_height_lp_hp_step(base,100.025,false);
assert(y(4)==0 && isnan(y(1)) && isnan(y(9)));
base(4)=1; base(1)=0.5; base(2)=0.5;
y=qdrone_height_lp_hp_step(base,100.05,false);
assert(y(1)==0.5 && y(9)==0 && y(3)==0,'Recovery poisoned by NaN');
y=qdrone_height_lp_hp_step(base,100.075,true);
assert(y(1)==0.5 && y(9)==0);
fprintf('PASS: LP attenuation, held DC height, HP DC rejection, invalid data, recovery, reset.\n');
fprintf('Synthetic 0.5 Hz +/-0.4 m input: LP peak %.4f m. Not a hardware accuracy guarantee.\n',max(abs(samples)));
end
