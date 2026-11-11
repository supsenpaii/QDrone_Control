function test_baro_height_kalman
% Offline checks only; no model or hardware is started.
folder = fileparts(mfilename('fullpath'));
addpath(folder,fullfile(folder,'capture_tool'));
clear qdrone_baro_height qdrone_baro_height_kalman;
p0 = 101325; dt = 0.025;
for k = 0:799
    y = qdrone_baro_height_kalman(p0,k*dt,0,9.80665,false);
end
assert(y(4)==0 && isnan(y(1)),'Reference locked before 20 seconds');
y = qdrone_baro_height_kalman(p0,20,0,9.80665,false);
assert(y(4)==1 && abs(y(1))<1e-8 && y(5)==p0);
raw = zeros(800,1); filtered = raw;
for k = 1:800
    h = 0.2*sin(2*pi*7*k*dt);
    p = p0*(1-h/44330)^5.25588;
    y = qdrone_baro_height_kalman(p,20+k*dt,0,9.80665,false);
    raw(k) = y(2); filtered(k) = y(1);
end
rawRms = sqrt(mean(raw(201:end).^2));
filteredRms = sqrt(mean(filtered(201:end).^2));
assert(filteredRms<rawRms/2,'Kalman failed stationary noise reduction');
for k = 1:800
    h = 0.1*k*dt;
    p = p0*(1-h/44330)^5.25588;
    y = qdrone_baro_height_kalman(p,40+k*dt,0.1,9.80665,false);
    assert(y(5)==p0,'Reference adapted during movement');
end
assert(abs(y(1)-2)<0.01 && abs(y(3)-0.1)<0.01,'Climb tracking failed');
for k = 1:2400
    y = qdrone_baro_height_kalman(p,60+k*dt,0,9.80665,false);
    assert(y(5)==p0,'Held height changed the reference');
end
assert(abs(y(1)-2)<1e-6,'Held height decayed');
y = qdrone_baro_height_kalman(NaN,120.025,0,9.80665,false);
assert(y(4)==0 && isnan(y(1)) && isnan(y(3)));
y = qdrone_baro_height_kalman(p,120.05,0,9.80665,false);
assert(y(4)==1 && abs(y(1)-2)<1e-8 && y(3)==0);
y = qdrone_baro_height_kalman(p,121,0,9.80665,false);
assert(y(4)==0 && y(8)==4);
y = qdrone_baro_height_kalman(p,121.025,0,9.80665,false);
assert(y(4)==1 && abs(y(1)-2)<1e-8);
y = qdrone_baro_height_kalman(p0,0,0,9.80665,false);
assert(y(4)==0 && isnan(y(5)),'Rewind did not clear reference');
for k = 1:800
    y = qdrone_baro_height_kalman(p0,k*dt,0,9.80665,false);
end
assert(y(4)==1);
y = qdrone_baro_height_kalman(p0,20.025,0,9.80665,true);
assert(y(4)==0 && isnan(y(5)));
for k = 1:820
    y = qdrone_baro_height_kalman(p0,20.025+k*dt,1,9.80665,false);
end
assert(y(4)==0 && y(8)==3,'Moving calibration accepted');
fprintf('PASS: 20s calibration, frozen reference, held height, climb, invalid data, gap, rewind, reset, movement.\n');
fprintf('PASS: synthetic noise RMS %.4f -> %.4f m.\n',rawRms,filteredRms);
end
