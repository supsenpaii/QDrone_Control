function test_baro_height
clear qdrone_baro_height;p0=101325;
for t=0:0.025:19.975,y=qdrone_baro_height(p0,t,0,9.80665,false);end
assert(y(4)==0&&isnan(y(5))&&y(6)<1,'Reference locked before 20 seconds');
y=qdrone_baro_height(p0,20,0,9.80665,false);
assert(y(4)==1&&y(1)==0&&abs(y(5)-p0)<1e-8,'Ground zero failed');
p1=p0*(1-1/44330)^5.25588;
for t=20.025:0.025:80
 y=qdrone_baro_height(p1,t,0.3,9.80665,false);
 assert(y(5)==p0,'Reference adapted after calibration');
end
assert(y(4)==1&&abs(y(1)-1)<1e-6,'Held height decayed with constant pressure');
% Changing pressure may change height, but must never change the ground reference.
for t=80.025:0.025:100
 y=qdrone_baro_height(p1+2*sin(t),t,0,9.80665,false);assert(y(5)==p0);
end
y=qdrone_baro_height(0,100.025,0,9.80665,false);assert(y(4)==0&&isnan(y(1))&&y(5)==p0);
y=qdrone_baro_height(p0,101,0,9.80665,false);assert(y(4)==0&&y(8)==4&&y(5)==p0);
y=qdrone_baro_height(p0,101.025,0,9.80665,false);assert(y(4)==1&&y(5)==p0);
y=qdrone_baro_height(p0,101.05,0,9.80665,true);assert(y(4)==0&&isnan(y(5)));
for t=101.075:0.025:125,y=qdrone_baro_height(p0,t,1,9.80665,false);end
assert(y(4)==0&&y(8)==3,'Moving calibration accepted');
for t=125.025:0.025:145.25,y=qdrone_baro_height(p0,t,0,9.80665,false);end
assert(y(4)==1);
y=qdrone_baro_height(p0-1000,145.275,0,9.80665,false);assert(y(4)==0&&y(8)==5&&y(5)==p0);
y=qdrone_baro_height(p0,0,0,9.80665,false);assert(y(4)==0&&isnan(y(5)));
% Confirm all 801 pressure samples contribute to the reference average.
clear qdrone_baro_height;expected=0;n=0;
for t=0:0.025:20
 p=100000+0.1*t;expected=expected+p;n=n+1;y=qdrone_baro_height(p,t,0,9.80665,false);
end
assert(y(4)==1&&abs(y(5)-expected/n)<1e-7,'Calibration interval was truncated');
fprintf('PASS: full 20-second average, frozen reference, held 1m for 60s, pressure drift without autozero, gaps, reset and movement rejection.\n');
end
