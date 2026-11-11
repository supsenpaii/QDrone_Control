function recorder=start_qdrone_capture(ip,recordFrames)
% Start host receiver only. Start QDrone_Capture target separately in QUARC.
if nargin<1,ip='192.168.2.52';end
if nargin<2,recordFrames=true;end
folder=fileparts(mfilename('fullpath'));addpath(folder);
existing=getappdata(0,'QDroneCaptureRecorder');
if ~isempty(existing) && isvalid(existing) && ~existing.Closing
    error('A capture receiver is already recording. Stop it first.');
end
recorder=QDroneCapture(ip,recordFrames);setappdata(0,'QDroneCaptureRecorder',recorder);
end
