function observer=start_height_observer(model)
% Open the height model if absent, then observe it in this MATLAB session.
% Never builds, starts, stops, or tunes a drone target.
if nargin<1,model='Read_IMU_Height';end
model=char(model);
folder=fileparts(mfilename('fullpath'));project=fileparts(folder);
existing=getappdata(0,'QDroneHeightObserver');
if isa(existing,'QDroneHeightObserver')&&isvalid(existing)&&~existing.Closing
    if strcmp(existing.Model,model)
        observer=existing;
        if isgraphics(observer.Figure),figure(observer.Figure);end
        fprintf('Height observer already running: %s\n',observer.Session);
        return;
    end
    error('QDrone:ObserverAlreadyRunning','An observer is recording another model. Close its observer window first.');
end
if ~bdIsLoaded(model)
    file=fullfile(project,[model '.slx']);
    if ~exist(file,'file')
        error('QDrone:HeightModelMissing','Height model file not found: %s',file);
    end
    open_system(file);
    fprintf('Opened model in this MATLAB session: %s\n',file);
end
subsystem=[model '/Ground Reference Height'];
if getSimulinkBlockHandle(subsystem)==-1
    error('QDrone:WrongHeightModel','Model %s does not contain Ground Reference Height. Open Read_IMU_Height.slx.',model);
end
open_system(subsystem);
observer=QDroneHeightObserver(model);setappdata(0,'QDroneHeightObserver',observer);
fprintf('Observing loaded file: %s\n',get_param(model,'FileName'));
state=get_param(model,'SimulationStatus');
if ~any(strcmp(state,{'running','external'}))
    fprintf('Model is %s. Observer waits for data; start Read_IMU_Height in QUARC after stopping other QDrone targets.\n',state);
end
end
