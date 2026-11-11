function y=qdrone_baro_height(p,t,gyroNorm,accelNorm,reset)
%#codegen
% BAROMETRIC relative height; no direct measurement of distance to the floor.
% Average 20 seconds of stationary raw pressure, then FREEZE p0.
% Only explicit reset or a new/rewound model clock can clear the reference.
% y=[filtered_m raw_m vertical_m_s valid reference_Pa progress noise_m status].
% status: 0 calibrating,1 valid,2 invalid pressure,3 moving during calibration,
% 4 time gap/rewind,5 pressure jump,6 noisy calibration rejected.
persistent p0 count pressureMean pressureM2 beginTime lastTick lastGood lastP filtered speed noise
if isempty(count)
 p0=NaN;count=0;pressureMean=0;pressureM2=0;beginTime=NaN;lastTick=NaN;
 lastGood=NaN;lastP=NaN;filtered=NaN;speed=0;noise=NaN;
end
rewind=isfinite(lastTick)&&isfinite(t)&&t<lastTick;
if reset || rewind
 p0=NaN;count=0;pressureMean=0;pressureM2=0;beginTime=NaN;lastTick=NaN;
 lastGood=NaN;lastP=NaN;filtered=NaN;speed=0;noise=NaN;
end
y=[NaN;NaN;NaN;0;p0;0;noise;0];
if ~isfinite(t)||(isfinite(lastTick)&&t<=lastTick),y(8)=4;return;end
previousTick=lastTick;lastTick=t;
if ~isfinite(p)||p<50000||p>110000
 if ~isfinite(p0),count=0;pressureMean=0;pressureM2=0;beginTime=NaN;end
 filtered=NaN;y(8)=2;return;
end
if isfinite(previousTick)&&t-previousTick>0.5
 filtered=NaN;lastGood=NaN;
 if ~isfinite(p0),count=0;pressureMean=0;pressureM2=0;beginTime=NaN;end
 y(8)=4;return;
end
if ~isfinite(p0)
 stationary=isfinite(gyroNorm)&&isfinite(accelNorm)&&gyroNorm<0.05&&accelNorm>8.8&&accelNorm<10.8;
 if ~stationary
  count=0;pressureMean=0;pressureM2=0;beginTime=NaN;y(8)=3;return;
 end
 if count==0,beginTime=t;end
 % Welford accumulation uses the ENTIRE calibration interval, not a capped array.
 count=count+1;delta=p-pressureMean;pressureMean=pressureMean+delta/count;
 pressureM2=pressureM2+delta*(p-pressureMean);
 elapsed=t-beginTime;y(6)=min(1,elapsed/20);
 if elapsed<20||count<200,return;end
 sigma=sqrt(max(0,pressureM2/max(1,count-1)));
 if sigma>4
  count=0;pressureMean=0;pressureM2=0;beginTime=NaN;y(6)=0;y(8)=6;return;
 end
 p0=pressureMean;noise=(44330/5.25588)*sigma/p0;
 filtered=0;speed=0;lastGood=t;lastP=p;
 raw=44330*(1-(p/p0)^(1/5.25588));
 y=[0;raw;0;1;p0;1;noise;1];return;
end
% From here p0 is read-only: no autozero and no adaptation at a held height.
raw=44330*(1-(p/p0)^(1/5.25588));
if isfinite(lastGood)&&t-lastGood<=0.5&&abs(p-lastP)>200
 filtered=NaN;y(5)=p0;y(8)=5;return;
end
if ~isfinite(filtered)||~isfinite(lastGood)||t-lastGood>0.5
 filtered=raw;speed=0;
else
 dt=t-lastGood;old=filtered;alpha=1-exp(-dt/0.5);
 filtered=filtered+alpha*(raw-filtered);
 speed=speed+(1-exp(-dt/1.0))*((filtered-old)/dt-speed);
end
lastGood=t;lastP=p;
y=[filtered;raw;speed;1;p0;1;noise;1];
end
