function report=configure_fa_flight(cameraFile,height_m,imageTop,stage,buildExecutable)
% Configure the EXISTING FA Stabilizer from measured camera data.
% configure_fa_flight() : read-only audit; no hardware connection.
% configure_fa_flight('cameraParams.mat',h,'nose','shadow',true)
% imageTop: nose/tail/left/right, describing where the TOP of image points.
% stage: shadow (disarmed diagnostic only), flight (actual bounded assistance).
% No download/Start/Arm. No change to RYPI gains or Mission Server.
folder=fileparts(mfilename('fullpath'));project=fileparts(folder);addpath(folder);
model='Stabilizer_QDrone_QGC_OF_RYPI_FA';load_system(fullfile(project,[model '.mdl']));
s=[model '/STABILIZER - QDRONE'];block=[s '/FA Configuration'];
current=str2num(get_param(block,'Value')); %#ok<ST2NM>
assert(numel(current)==13,'Unexpected FA configuration.');
if nargin==0
    report=struct('model',model,'stage',current(1),'fx_fy',current(2:3), ...
        'nominal_optical_depth_m',current(4),'simulation_status',get_param(model,'SimulationStatus'), ...
        'camera_measurements_present',all(current(2:4)>0),'flight_validated',false);
    disp(report);
    if ~report.camera_measurements_present
        fprintf('BLOCKED: measured fx/fy, nominal optical depth, and mounting direction are missing.\n');
    end
    return;
end
assert(nargin>=4,'Supply camera MAT file, measured depth, image-top direction, and stage.');
if nargin<5,buildExecutable=false;end
assert(strcmp(get_param(model,'SimulationStatus'),'stopped'),'Disarm and Stop Stabilizer first.');
assert(strcmp(get_param(model,'Dirty'),'off'),'Save Stabilizer first; the saved file is backed up.');
validateattributes(height_m,{'numeric'},{'real','finite','scalar','>=',.3,'<=',3});
assert(isfile(cameraFile),'Camera parameter file does not exist.');
data=load(cameraFile,'cameraParams');assert(isfield(data,'cameraParams'),'MAT must contain cameraParams.');
p=data.cameraParams;
assert(isa(p,'cameraParameters')||isa(p,'cameraIntrinsics')||isstruct(p),'Unsupported camera parameters.');
if isstruct(p),assert(isfield(p,'FocalLength')&&isfield(p,'ImageSize'),'Need FocalLength and ImageSize.');end
f=double(p.FocalLength);imageSize=double(p.ImageSize);
assert(isequal(imageSize(:)',[480 640]),'Calibration must match the actual 640x480 flow image.');
assert(numel(f)==2&&all(isfinite(f))&&all(f>=50)&&all(f<=3000),'Invalid focal lengths in pixels.');
switch lower(char(imageTop))
    case 'nose',heading=0;
    case 'left',heading=pi/2;
    case 'tail',heading=pi;
    case 'right',heading=-pi/2;
    otherwise,error('Use nose/tail/left/right for the measured TOP of image direction.');
end
R=[sin(heading) -cos(heading) 0;-cos(heading) -sin(heading) 0;0 0 -1];
switch lower(char(stage))
    case 'shadow',stageValue=2;
    case 'flight',stageValue=1;
    otherwise,error('Stage must be shadow or flight.');
end
cfg=current(:);cfg(1:4)=[stageValue;f(:);height_m];
if stageValue==1
    evidence=fullfile(folder,'fa_shadow_verification.mat');
    assert(isfile(evidence),'First configure shadow and verify a real Disarmed known-motion log.');
    checked=load(evidence,'verification');assert(isfield(checked,'verification'),'Missing ground verification.');
    v=checked.verification;
    assert(v.passed&&v.ground_only&&max(abs(v.config(2:end)-cfg(2:end)))<1e-12 ...
        &&max(abs(v.cameraToBody(:)-R(:)))<1e-12, ...
        'Ground verification must match the measured camera, height, mounting and controller configuration.');
end
% Exercise the actual shared MEX with these parameters before editing model.
clear qdrone_fa_step
cmd=[0;0;0];flow=[0;0;.8;1;.033;0;30;28];ft=0;
arm=double(stageValue==1);throttle=10*arm;
for k=1:1200
    t=k*.001;if mod(k,33)==0,ft=t;end
    [out,d]=qdrone_fa_step(cmd,flow,ft,zeros(3,1),zeros(3,1),arm,throttle,0,1,cfg,R,t);
end
assert(d(15)==1&&d(1)==(5+4*double(stageValue==2)),'Shared MEX rejects this config/stage. Rebuild MEX if old.');
assert(isequal(out,cmd),'Zero flow should not produce an offset in this synthetic check.');
clear qdrone_fa_step
file=get_param(model,'FileName');copyfile(file,[file '.' datestr(now,'yyyymmdd_HHMMSS') '.profile.bak']);
set_param(block,'Value',mat2str(cfg,17));set_param([s '/FA Camera To Body'],'Value',mat2str(R,17));
set_param(model,'DownloadToTarget','off');set_param(model,'SimulationCommand','update');save_system(model);
profile=struct('cameraFile',cameraFile,'height_m',height_m,'imageTop',imageTop,'stage',stage,'config',cfg,'cameraToBody',R, ...
    'flight_validated',false,'download_start_arm_performed',false);
save(fullfile(folder,'fa_measured_profile.mat'),'profile');
if buildExecutable,slbuild(model);fprintf('PASS: FA %s local build. No download, Start or Arm.\n',char(stage));end
report=profile;disp(report);
end
