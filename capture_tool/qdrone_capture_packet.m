function packet=qdrone_capture_packet(t,analog,sensors,flow,frameSeq,frameTime)
%#codegen
% 30 doubles; see capture_schema.json written by build_qdrone_capture.
% Keep the schema fixed for six-channel IMU and legacy full-sensor models.
sensorValues=nan(11,1);
assert(numel(sensors)==6 || numel(sensors)==11);
sensorValues(1:numel(sensors))=sensors(:);
packet=[51234567;1;round(t/0.001);t;analog(:);sensorValues;flow(:);frameSeq;frameTime];
end
