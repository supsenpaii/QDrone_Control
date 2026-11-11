classdef QDroneHeightObserver < handle
    % Read host-side viewing blocks; never starts/stops or tunes the target.
    properties
        Model; Session; Timer; Figure; StatusText; PlotLine; StartClock;
        Closing=false; LastTime=NaN; LastAdvance=-Inf; LastGoodRead=-Inf;
        Values=nan(8,1); Epoch=0; Count=0; History=zeros(0,2); LastError='';
        LastPublish=-Inf; LastState='waiting';
    end
    methods
        function self=QDroneHeightObserver(model)
            if nargin<1,model='Read_IMU_Height';end
            assert(bdIsLoaded(model),'Open the running height model first.');
            self.Model=model;self.StartClock=tic;
            project=fileparts(fileparts(mfilename('fullpath')));root=fullfile(project,'height_sessions');
            if ~exist(root,'dir'),mkdir(root);end
            [~,id]=fileparts(tempname);self.Session=fullfile(root,[datestr(now,'yyyymmdd_HHMMSS') '_' id]);mkdir(self.Session);
            file=fopen(fullfile(self.Session,'height.csv'),'w');assert(file>=0);
            fprintf(file,'host_utc_s,target_s,height_filtered_m,height_raw_m,vertical_speed_m_s,valid,reference_Pa,calibration_progress,calibration_noise_m,status_code,epoch\n');fclose(file);
            self.writeJson(fullfile(root,'active_session.json'),struct('session',self.Session,'model',model));
            self.Figure=figure('Name','Barometric relative height - observer only','NumberTitle','off', ...
                'Position',[100 100 1000 580],'CloseRequestFcn',@(~,~)self.stop);
            ax=axes('Parent',self.Figure,'Units','normalized','Position',[0.08 0.19 0.55 0.73]);
            self.PlotLine=plot(ax,NaN,NaN,'LineWidth',1.5);grid(ax,'on');
            xlabel(ax,'Target time (s)');ylabel(ax,'Relative barometric height (m)');
            self.StatusText=uicontrol(self.Figure,'Style','text','Units','normalized','Position',[0.67 0.2 0.31 0.72], ...
                'HorizontalAlignment','left','FontSize',11);
            uicontrol(self.Figure,'Style','pushbutton','Units','normalized','Position',[0.68 0.06 0.25 0.08], ...
                'String','STOP OBSERVER','Callback',@(~,~)self.stop);
            self.Timer=timer('ExecutionMode','fixedSpacing','Period',0.1,'BusyMode','drop','TimerFcn',@(~,~)self.tick);
            self.publish('waiting',false);
            start(self.Timer);
            fprintf('Height observer session: %s\n',self.Session);
            fprintf('The existing target is unchanged. Observer samples viewing blocks at up to 10 Hz.\n');
        end
        function tick(self)
            if self.Closing,return;end
            now=toc(self.StartClock);
            try
                state=get_param(self.Model,'SimulationStatus');
                if ~any(strcmp(state,{'running','external'}))
                    self.LastState=['model_' state];
                else
                    labels={'Relative Height m','Raw Height m','Vertical Speed m_s','Height Valid', ...
                        'Ground Pressure Pa','Calibration Progress','Calibration Noise m','Status Code'};
                    y=nan(8,1);times=nan(8,1);
                    for k=1:8
                        block=[self.Model '/Ground Reference Height/' labels{k}];
                        runtime=get_param(block,'RuntimeObject');
                        if isempty(runtime),error('QDrone:NoRuntime','Live viewing-block data unavailable at %s',block);end
                        value=runtime.InputPort(1).Data;
                        assert(isnumeric(value)||islogical(value),'Viewing block returned unsupported data');
                        assert(isscalar(value),'Viewing block data is not scalar');y(k)=double(value);
                        try,times(k)=double(runtime.CurrentTime);catch,end
                    end
                    t=double(get_param(self.Model,'SimulationTime'));
                    if any(isfinite(times)),t=max([t;times(isfinite(times))]);end
                    if ~isfinite(t),error('QDrone:NoTime','No finite target/viewing-block time.');end
                    if isfinite(self.LastTime)&&t<self.LastTime
                        self.Epoch=self.Epoch+1;self.History=zeros(0,2);
                    end
                    advanced=~isfinite(self.LastTime)||t~=self.LastTime;
                    if advanced
                        self.LastAdvance=now;self.LastTime=t;self.Values=y;self.Count=self.Count+1;
                        row=[self.utc t y' self.Epoch];
                        f=fopen(fullfile(self.Session,'height.csv'),'a');assert(f>=0,'Cannot append height CSV');
                        fprintf(f,[repmat('%.12g,',1,10) '%.12g\n'],row);fclose(f);
                        if y(4)>0.5 && isfinite(y(1))
                            self.History(end+1,:)=[t y(1)];
                            self.History=self.History(self.History(:,1)>=t-60,:);
                        end
                    end
                    self.LastGoodRead=now;self.LastError='';self.LastState=state;
                end
            catch cause
                self.LastError=cause.message;self.LastState='read_error';
            end
            if now-self.LastPublish>=0.5
                fresh=now-self.LastAdvance<0.75 && now-self.LastGoodRead<0.75 && any(strcmp(self.LastState,{'running','external'}));
                self.publish(self.LastState,fresh);self.LastPublish=now;
            end
        end
        function publish(self,state,fresh)
            y=self.Values;valid=fresh && ~self.Closing && y(4)>0.5 && isfinite(y(1)) && isfinite(y(5));
            reason='Waiting for live height data';
            if strcmp(state,'read_error'),reason=['Cannot read live viewing blocks: ' self.LastError];
            elseif ~fresh,reason='Data not fresh or model is stopped/paused';
            elseif ~valid
                if y(8)==2,reason='Invalid raw pressure - do not use bias-removed pressure';
                elseif y(8)==3,reason='Keep drone still on the ground to establish zero';
                elseif y(8)==4,reason='Target time gap/reset';
                elseif y(8)==5,reason='Pressure jump rejected'; elseif y(8)==6,reason='Calibration pressure is too noisy; keep still and wait';
                else,reason='Calibrating ground reference - keep still for 20 seconds';end
            else
                reason='Barometric estimate only; accuracy not verified against a ruler';
            end
            height=NaN;speed=NaN;if valid,height=y(1);speed=y(3);end
            status=struct('session',self.Session,'model',self.Model,'recording',~self.Closing,'host_utc_s',self.utc, ...
                'target_s',self.LastTime,'epoch',self.Epoch,'samples',self.Count,'data_fresh',fresh&&~self.Closing, ...
                'height_valid',valid,'height_relative_barometric_m',height,'height_raw_m',y(2), ...
                'vertical_speed_m_s',speed,'ground_reference_pressure_Pa',y(5),'calibration_progress',y(6), ...
                'calibration_noise_equivalent_m',y(7),'status_code',y(8),'state',state,'last_error',self.LastError, ...
                'measurement_type','barometric relative to calibrated reference; not direct AGL range', ...
                'range_sensor_available',false,'accuracy_verified',false,'assessment',reason);
            self.writeJson(fullfile(self.Session,'latest.json'),status);
            if isgraphics(self.StatusText)
                set(self.StatusText,'String',sprintf(['%s\n\nEstimated baro height: %.3f m\nVertical speed: %.3f m/s\n' ...
                    'Calculation valid: %d\nFresh data: %d\nReference: %.1f Pa\nCalibration: %.0f %%\n' ...
                    'Calibration noise equivalent: %.3f m\nStatus code: %.0f\nSamples: %d\n\nObserver only - no target control'], ...
                    reason,height,speed,valid,fresh,y(5),100*y(6),y(7),y(8),self.Count));
            end
            if isgraphics(self.PlotLine)&&~isempty(self.History)
                set(self.PlotLine,'XData',self.History(:,1),'YData',self.History(:,2));
            end
            drawnow limitrate;
        end
        function stop(self)
            if self.Closing,return;end
            self.Closing=true;
            if ~isempty(self.Timer)&&isvalid(self.Timer),stop(self.Timer);delete(self.Timer);end
            try,self.publish('observer_stopped',false);catch,end
            if isgraphics(self.Figure),delete(self.Figure);end
            fprintf('Height observer stopped. CSV saved in %s\n',self.Session);
        end
    end
    methods(Static)
        function t=utc,t=posixtime(datetime('now','TimeZone','UTC'));end
        function writeJson(path,status)
            temp=[path '.tmp'];f=fopen(temp,'w');assert(f>=0,'Cannot write height status');
            fprintf(f,'%s',jsonencode(status));fclose(f);[ok,message]=movefile(temp,path,'f');assert(ok,message);
        end
    end
end


