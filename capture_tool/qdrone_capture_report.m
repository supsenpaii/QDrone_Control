function report=qdrone_capture_report(session)
path=fullfile(session,'telemetry.csv');
if ~exist(path,'file'),error('No telemetry.csv in session');end
v=readmatrix_compat(path);
report=struct('samples',size(v,1),'position_hold_validated',false);
if isempty(v)
    report.assessment='No telemetry was received. Check target and network.';
else
    epoch=v(:,32);dt=diff(v(:,4));same=diff(epoch)==0;
    report.target_span_s=v(end,4)-v(1,4);report.epochs=numel(unique(epoch));
    report.median_sample_interval_s=median(dt(same & dt>0));
    report.battery_min_v=min(v(:,9));report.battery_max_v=max(v(:,9));
    report.target_sequence_gaps=sum(max(0,diff(v(:,3))-1).*same);
    [~,index]=unique([epoch v(:,29)],'rows','last');flow=v(index,:);flow=flow(flow(:,29)>0,:);
    report.camera_frames_in_telemetry=size(flow,1);
    report.flow_valid_fraction=mean(flow(:,24)>0.5);
    good=flow(:,24)>0.5;
    if any(good)
        report.flow_dt_median_s=median(flow(good,25));report.flow_quality_median=median(flow(good,23));
    end
    report.gyro_rms_rad_s=sqrt(mean(sum(v(:,10:12).^2,2)));
    report.accel_norm_median_m_s2=median(sqrt(sum(v(:,13:15).^2,2)));
    report.assessment='Data collection only. Handheld motion does not validate flight-control gains.';
end
QDroneCapture.writeJson(fullfile(session,'report.json'),report);
f=fopen(fullfile(session,'REPORT.txt'),'w');assert(f>=0);
fprintf(f,'QDRONE SENSOR CAPTURE REPORT\nSession: %s\nSamples: %d\n',session,report.samples);
if ~isempty(v)
    fprintf(f,'Target sequence gaps: %d\nFlow valid fraction: %.1f %%\n',report.target_sequence_gaps,100*report.flow_valid_fraction);
    fprintf(f,'Gyro RMS: %.6f rad/s\nAccel norm median: %.4f m/s^2\n',report.gyro_rms_rad_s,report.accel_norm_median_m_s2);
end
fprintf(f,'%s\n',report.assessment);fclose(f);
end
function v=readmatrix_compat(path)
f=fopen(path,'r');assert(f>=0);cleanup=onCleanup(@()fclose(f)); %#ok<NASGU>
header=fgetl(f); %#ok<NASGU>
values=textscan(f,repmat('%f',1,32),'Delimiter',',','CollectOutput',true);
v=values{1};
end
