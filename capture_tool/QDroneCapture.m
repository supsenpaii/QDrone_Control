classdef QDroneCapture < handle
    % Sensor-only host receiver. Never sends a control command to the drone.
    properties
        Session; Root; IP; Figure; StatusText; Preview; StageControl; Timer;
        Streams; Pending; Connected; RetryAt; Buffers;
        CameraLast; CameraCount; CameraSeq; CameraBytes; CameraPreviewAt;
        LastTelemetry=-1; LastTarget=NaN; LastSeq=NaN; Latest=zeros(30,1);
        Received=0; Dropped=0; Bad=0; Epoch=0; Window=zeros(30,0);
        StartClock; LastStatus=-1; Closing=false; ReceiveFrames=true;
        Fatal=''; LastErrors; Stage='stationary'; MarkerDistance; MarkerHeight;
    end
    methods
        function self=QDroneCapture(ip,recordFrames)
            if nargin<1,ip='192.168.2.52';end
            if nargin<2,recordFrames=true;end
            self.IP=ip;self.ReceiveFrames=logical(recordFrames);
            self.Root=fileparts(fileparts(mfilename('fullpath')));
            base=fullfile(self.Root,'capture_sessions');if ~exist(base,'dir'),mkdir(base);end
            [~,uid]=fileparts(tempname);name=[datestr(now,'yyyymmdd_HHMMSS') '_' uid];
            self.Session=fullfile(base,name);mkdir(self.Session);
            self.Streams=cell(1,6);self.Pending=false(1,6);self.Connected=false(1,6);
            self.RetryAt=zeros(1,6);self.Buffers=cell(1,6);self.LastErrors=repmat({''},1,6);
            for k=1:6,self.Buffers{k}=[];end
            self.CameraLast=-ones(1,5);self.CameraCount=zeros(1,5);self.CameraSeq=zeros(1,5);
            self.CameraBytes=zeros(1,5);self.CameraPreviewAt=-ones(1,5);
            self.StartClock=tic;
            headers={'magic','schema','target_seq','target_s','analog0','analog1','analog2','analog3','battery_v', ...
                'gyro_x_rad_s','gyro_y_rad_s','gyro_z_rad_s','accel_x_m_s2','accel_y_m_s2','accel_z_m_s2', ...
                'mag_x_T','mag_y_T','mag_z_T','pressure_Pa','temperature_C', ...
                'du_px','dv_px','flow_quality','flow_valid','frame_dt_s','flow_age_s','textured','inliers','down_frame_seq','down_frame_target_s'};
            f=fopen(fullfile(self.Session,'telemetry.csv'),'w');assert(f>=0);fprintf(f,'%s,host_receive_utc_s,epoch\n',strjoin(headers,','));fclose(f);
            f=fopen(fullfile(self.Session,'events.csv'),'w');fprintf(f,'host_utc_s,target_s,epoch,stage,distance_m,height_m\n');fclose(f);
            f=fopen(fullfile(self.Session,'frames.csv'),'w');fprintf(f,'camera_id,frame_seq,target_s,host_receive_utc_s,epoch,byte_offset,payload_bytes,height,width,channels,bytes_per_element\n');fclose(f);
            config=struct('ip',ip,'record_frames',self.ReceiveFrames,'motor_outputs_enabled',false, ...
                'timestamp_note','Target times share one model clock. Camera stamp is software read time, not exposure time.', ...
                'unavailable',{{'range','motor RPM','FrSky commands','ground truth position'}});
            schema=fullfile(self.Root,'capture_tool','capture_schema.json');
            if exist(schema,'file'),config.target=jsondecode(fileread(schema));end
            self.writeJson(fullfile(self.Session,'session.json'),config);
            self.writeJson(fullfile(base,'active_session.json'),struct('session',self.Session,'started_utc',self.utc));
            self.makeUI;
            self.Timer=timer('ExecutionMode','fixedSpacing','Period',0.05,'BusyMode','drop', ...
                'TimerFcn',@(~,~)self.tick,'ErrorFcn',@(~,e)self.timerError(e));
            self.publish;start(self.Timer);
            fprintf('Capture session: %s\n',self.Session);
            fprintf('Live status: %s\n',fullfile(self.Session,'latest.json'));
        end
        function makeUI(self)
            self.Figure=figure('Name','QDrone sensor capture - NO MOTOR OUTPUTS','NumberTitle','off', ...
                'Position',[80 80 1120 720],'CloseRequestFcn',@(~,~)self.stop);
            axesHandle=axes('Parent',self.Figure,'Units','normalized','Position',[0.03 0.25 0.57 0.7]);
            self.Preview=imagesc(zeros(480,640,'uint8'),'Parent',axesHandle);axis(axesHandle,'image');colormap(axesHandle,gray(256));
            title(axesHandle,'Down camera - raw image (display only)');
            self.StatusText=uicontrol(self.Figure,'Style','text','Units','normalized', ...
                'Position',[0.63 0.28 0.35 0.67],'HorizontalAlignment','left','FontSize',11);
            uicontrol(self.Figure,'Style','text','Units','normalized','Position',[0.03 0.16 0.12 0.04],'String','Test stage:');
            self.StageControl=uicontrol(self.Figure,'Style','popupmenu','Units','normalized','Position',[0.15 0.16 0.26 0.045], ...
                'String',{'stationary','translate_forward','translate_right','rotate_roll','rotate_pitch','rotate_yaw','raise_lower','lighting','end'});
            uicontrol(self.Figure,'Style','pushbutton','Units','normalized','Position',[0.43 0.16 0.16 0.05], ...
                'String','MARK STAGE','Callback',@(~,~)self.mark);
            uicontrol(self.Figure,'Style','text','Units','normalized','Position',[0.03 0.09 0.12 0.04],'String','Distance m:');
            self.MarkerDistance=uicontrol(self.Figure,'Style','edit','Units','normalized','Position',[0.15 0.09 0.12 0.045],'String','NaN');
            uicontrol(self.Figure,'Style','text','Units','normalized','Position',[0.30 0.09 0.12 0.04],'String','Height m:');
            self.MarkerHeight=uicontrol(self.Figure,'Style','edit','Units','normalized','Position',[0.43 0.09 0.12 0.045],'String','NaN');
            uicontrol(self.Figure,'Style','pushbutton','Units','normalized','Position',[0.66 0.10 0.25 0.07], ...
                'String','STOP RECORDING','Callback',@(~,~)self.stop);
        end
        function mark(self)
            items=get(self.StageControl,'String');self.Stage=items{get(self.StageControl,'Value')};
            d=str2double(get(self.MarkerDistance,'String'));h=str2double(get(self.MarkerHeight,'String'));
            f=fopen(fullfile(self.Session,'events.csv'),'a');assert(f>=0);
            fprintf(f,'%.6f,%.9f,%d,%s,%.6f,%.6f\n',self.utc,self.LastTarget,self.Epoch,self.Stage,d,h);fclose(f);
        end
        function tick(self)
            if self.Closing,return;end
            now=toc(self.StartClock);
            for channel=1:6
                if channel>1 && ~self.ReceiveFrames,continue;end
                if isempty(self.Streams{channel})
                    if now<self.RetryAt(channel),continue;end
                    try
                        [self.Streams{channel},self.Pending(channel)]=stream_connect(sprintf('tcpip://%s:%d',self.IP,18909+channel),true,2097152);
                        self.Connected(channel)=~self.Pending(channel);
                    catch cause,self.LastErrors{channel}=cause.message;self.RetryAt(channel)=now+2;continue;end
                end
                try
                    if self.Pending(channel)
                        if ~any(stream_poll(self.Streams{channel},0,'connect')),continue;end
                        self.Pending(channel)=false;self.Connected(channel)=true;
                    end
                    if channel==1,template=zeros(30000,1);else,template=zeros(1048576,1,'uint8');end
                    [data,wouldBlock]=stream_receive(self.Streams{channel},template);
                    if isempty(data)
                        if ~wouldBlock,self.disconnect(channel,now,'Peer closed connection');end
                        continue;
                    end
                    self.LastErrors{channel}='';
                    self.Buffers{channel}=[self.Buffers{channel}(:);data(:)];
                    if channel==1
                        [packets,self.Buffers{channel},bad]=qdrone_parse_telemetry(self.Buffers{channel});self.Bad=self.Bad+bad;
                        self.acceptTelemetry(packets,now);
                    else
                        [frames,self.Buffers{channel},bad]=qdrone_parse_frames(self.Buffers{channel});self.Bad=self.Bad+bad;
                        self.acceptFrames(frames,now);
                    end
                    if numel(self.Buffers{channel})>4000000
                        error('QDrone:Protocol','Receive buffer exceeded framing limit');
                    end
                catch cause,self.disconnect(channel,now,cause.message);end
            end
            if now-self.LastStatus>=1,self.publish;self.LastStatus=now;end
        end
        function disconnect(self,channel,now,message)
            try,stream_close(self.Streams{channel});catch,end
            self.Streams{channel}=[];self.Pending(channel)=false;self.Connected(channel)=false;
            self.Buffers{channel}=[];self.RetryAt(channel)=now+2;self.LastErrors{channel}=message;
        end
        function acceptTelemetry(self,p,now)
            if isempty(p),return;end
            rows=zeros(size(p,2),32);host=self.utc;
            for k=1:size(p,2)
                v=p(:,k);
                if isfinite(self.LastTarget) && v(4)<self.LastTarget
                    self.Epoch=self.Epoch+1;self.LastSeq=NaN;self.Window=zeros(30,0);
                end
                if isfinite(self.LastSeq),self.Dropped=self.Dropped+max(0,v(3)-self.LastSeq-1);end
                self.LastSeq=v(3);self.LastTarget=v(4);self.Latest=v;rows(k,:)=[v' host self.Epoch];
            end
            self.Received=self.Received+size(p,2);self.LastTelemetry=now;
            self.Window=[self.Window p];self.Window=self.Window(:,self.Window(4,:)>=self.LastTarget-2);
            if size(self.Window,2)>4000,self.Window=self.Window(:,end-3999:end);end
            f=fopen(fullfile(self.Session,'telemetry.csv'),'a');assert(f>=0);
            cleanup=onCleanup(@()fclose(f));format=[repmat('%.12g,',1,31) '%.12g\n'];fprintf(f,format,rows'); %#ok<NASGU>
        end
        function acceptFrames(self,frames,now)
            host=self.utc;
            for k=1:numel(frames)
                frame=frames{k};h=frame.header;id=h(1);
                if h(2)<=0 || h(2)==self.CameraSeq(id),continue;end
                self.CameraSeq(id)=h(2);self.CameraCount(id)=self.CameraCount(id)+1;self.CameraLast(id)=now;
                path=fullfile(self.Session,sprintf('camera_%d.bin',id));
                f=fopen(path,'a','ieee-le');assert(f>=0);offset=ftell(f);
                count=fwrite(f,frame.pixels,'uint8');fclose(f);assert(count==numel(frame.pixels),'Camera disk write incomplete');
                self.CameraBytes(id)=self.CameraBytes(id)+count;
                f=fopen(fullfile(self.Session,'frames.csv'),'a');assert(f>=0);
                fprintf(f,'%d,%.0f,%.9f,%.6f,%d,%.0f,%d,%d,%d,%d,%d\n',id,h(2),h(3),host,self.Epoch,offset,count,h(4),h(5),h(6),h(7));fclose(f);
                if now-self.CameraPreviewAt(id)>=1
                    if h(7)==2,pixels=typecast(frame.pixels,'uint16');else,pixels=frame.pixels;end
                    image=reshape(pixels,[h(4) h(5) h(6)]);
                    if id==3,preview=uint8(min(double(image)/4000,1)*255);else,preview=image;end
                    tmp=fullfile(self.Session,sprintf('preview_%d.tmp.png',id));
                    imwrite(preview,tmp);movefile(tmp,fullfile(self.Session,sprintf('latest_camera_%d.png',id)),'f');
                    self.CameraPreviewAt(id)=now;
                    if id==1 && isgraphics(self.Preview),set(self.Preview,'CData',preview);end
                end
            end
        end
        function publish(self)
            now=toc(self.StartClock);v=self.Latest;
            age=Inf;if self.LastTelemetry>=0,age=now-self.LastTelemetry;end
            live=age<0.5 && self.Received>0 && ~self.Closing;
            goodFraction=0;quality=0;gyroRms=NaN;accelNorm=NaN;
            if ~isempty(self.Window)
                % Count unique camera frames, not the 1 kHz held copies.
                [~,index]=unique(self.Window(29,:),'last');flow=self.Window(:,index);flow=flow(:,flow(29,:)>0);
                if ~isempty(flow),goodFraction=mean(flow(24,:)>0.5);quality=median(flow(23,:));end
                gyroRms=sqrt(mean(sum(self.Window(10:12,:).^2,1)));
                accelNorm=median(sqrt(sum(self.Window(13:15,:).^2,1)));
            end
            judgement='NO DATA - wait for target connection';
            if self.Received>0 && ~live,judgement='STALE / DISCONNECTED - do not evaluate old data';
            elseif live
                if goodFraction<0.7,judgement='FLOW WEAK - slow movement, improve floor texture/light';
                else,judgement='FLOW AVAILABLE - image motion only; no position hold';end
                if strcmp(self.Stage,'stationary') && gyroRms>0.05,judgement='STATIONARY TEST: movement detected; place drone still';end
            end
            status=struct('schema',1,'session',self.Session,'host_utc_s',self.utc,'recording',~self.Closing, ...
                'telemetry_live',live,'telemetry_age_s',age,'target_s',self.LastTarget,'epoch',self.Epoch, ...
                'samples_received',self.Received,'samples_dropped',self.Dropped,'bad_framing_elements',self.Bad, ...
                'stage',self.Stage,'battery_v',v(9),'gyro_rad_s',v(10:12)', ...
                'accel_m_s2',v(13:15)','mag_T',v(16:18)','pressure_Pa',v(19),'temperature_C',v(20), ...
                'flow_du_dv_px',v(21:22)','flow_valid',live && v(24)>0.5 && v(4)-v(30)<0.15, ...
                'flow_valid_fraction_2s',goodFraction,'flow_quality_median_2s',quality, ...
                'frame_dt_s',v(25),'gyro_rms_rad_s',gyroRms,'accel_norm_m_s2',accelNorm, ...
                'camera_frames_saved',self.CameraCount,'camera_last_sequence',self.CameraSeq, ...
                'camera_age_s',now-self.CameraLast,'camera_bytes',self.CameraBytes, ...
                'network_connected',self.Connected,'network_errors',{self.LastErrors}, ...
                'motor_outputs_enabled',false,'range_available',false,'assessment',judgement,'fatal_error',self.Fatal);
            self.writeJson(fullfile(self.Session,'latest.json'),status);
            if isgraphics(self.StatusText)
                text=sprintf(['%s\n\nStage: %s\nTelemetry age: %.2f s\nSamples: %d | gaps: %d\n' ...
                    'Battery: %.2f V\nGyro RMS: %.4f rad/s\nAcceleration norm: %.3f m/s^2\n' ...
                    'Flow valid (2s): %.1f %%\nQuality median: %.3f\nFrame dt: %.4f s\n' ...
                    'Images saved [Down RGB Depth L R]:\n%s\n\nNO MOTOR OUTPUTS / NO POSITION HOLD'], ...
                    judgement,self.Stage,age,self.Received,self.Dropped,v(9),gyroRms,accelNorm,100*goodFraction,quality,v(25),mat2str(self.CameraCount));
                set(self.StatusText,'String',text);
            end
            drawnow limitrate;
        end
        function timerError(self,event)
            try,self.Fatal=event.Data.Message;catch,self.Fatal='Receiver timer error';end
            self.stop;
        end
        function stop(self)
            if self.Closing,return;end
            self.Closing=true;
            if ~isempty(self.Timer) && isvalid(self.Timer),stop(self.Timer);delete(self.Timer);end
            for k=1:6
                if ~isempty(self.Streams{k}),try,stream_close(self.Streams{k});catch,end,end
            end
            try,self.publish;catch,end
            try,qdrone_capture_report(self.Session);catch cause,fprintf('Report error: %s\n',cause.message);end
            if isgraphics(self.Figure),delete(self.Figure);end
            fprintf('Recording stopped. Session: %s\n',self.Session);
            fprintf('Target is sensor-only; stop QDrone_Capture separately in QUARC.\n');
        end
    end
    methods(Static)
        function t=utc
            t=(now-datenum(1970,1,1))*86400;
            % now is local wall time: use timezone-aware datetime to convert to UTC.
            t=posixtime(datetime('now','TimeZone','UTC'));
        end
        function writeJson(path,data)
            tmp=[path '.tmp'];f=fopen(tmp,'w');assert(f>=0,'Cannot write status file');
            try,fprintf(f,'%s',jsonencode(data));fclose(f);catch cause,fclose(f);rethrow(cause);end
            [ok,msg]=movefile(tmp,path,'f');assert(ok,msg);
        end
    end
end


