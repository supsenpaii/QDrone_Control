function report=configure_flow_assist_switch(buttonIndex)
% Inspect/open FA switch controls, or assign a VERIFIED free RC button.
% No build/download/Start/Arm. Keep calibration lock unchanged.
% configure_flow_assist_switch       : inspect and show displays
% configure_flow_assist_switch(0)    : disable switch selection
% configure_flow_assist_switch(N)    : map observed free button N (3..32)
folder=fileparts(mfilename('fullpath'));project=fileparts(folder);
mission='Mission_Server_QDrone_Manual_Flight_FrSky_FA';
stabilizer='Stabilizer_QDrone_QGC_OF_RYPI_FA';
load_system(fullfile(project,[mission '.mdl']));
ms=[mission '/MISSION SERVER QDRONE MANUAL FLIGHT'];
indexBlock=[ms '/FA Button Index'];
baseBlock=[ms '/' sprintf('Manual Flight Mode\n1 - Torque Mode,   0 - Angle Mode')];
sbInstalled=getSimulinkBlockHandle([ms '/FA SB Enabled'])~=-1;
assert(nargin==0||~sbInstalled, ...
    'This model uses SB Rz axis. Do not assign a button index; use setup_sb_flow_assist.');
if nargin>0
    assert(isnumeric(buttonIndex)&&isreal(buttonIndex)&&isscalar(buttonIndex)&&isfinite(buttonIndex) ...
        &&buttonIndex==floor(buttonIndex)&&(buttonIndex==0||(buttonIndex>=3&&buttonIndex<=32)), ...
        'Use 0 to disable, or a measured free button index 3..32. Buttons 1/2 are reserved.');
    assert(strcmp(get_param(mission,'SimulationStatus'),'stopped'), ...
        'Disarm and Stop Mission Server before changing its RC mapping.');
    assert(~bdIsLoaded(stabilizer)||strcmp(get_param(stabilizer,'SimulationStatus'),'stopped'), ...
        'Stop the FA Stabilizer before changing its RC mapping.');
    assert(strcmp(get_param(mission,'Dirty'),'off'), ...
        'Save the current Mission Server first; this helper backs up the saved file.');
    if buttonIndex~=0
        assert(str2double(get_param(baseBlock,'Value'))==0, ...
            'Base mode must already be Angle (0). Do not use Torque (1) for Flow Assist.');
    end
    file=get_param(mission,'FileName');
    copyfile(file,[file '.' datestr(now,'yyyymmdd_HHMMSS') '.switch.bak']);
    set_param(indexBlock,'Value',num2str(buttonIndex));save_system(mission);
    fprintf('Saved FA button index=%d. Rebuild/download ONLY after verifying the paired FA models.\n',buttonIndex);
end
report=struct;
report.mission=mission;
report.simulation_status=get_param(mission,'SimulationStatus');
report.button_index=str2double(get_param(indexBlock,'Value'));
report.base_mode=str2double(get_param(baseBlock,'Value'));
report.download_on_build=get_param(mission,'DownloadToTarget');
report.executable_exists=isfile(fullfile(project,[mission '.rt-win64']));
report.mapping_enabled=report.button_index>=3&&report.button_index<=32&&report.base_mode==0;
report.sb_axis_installed=sbInstalled;
report.calibration_changed=false;report.download_start_arm_performed=false;
disp(report);
open_system(ms);
if sbInstalled
    hilite_system([ms '/FA SB Raw -1 Up 0 Middle 1 Down'],'find');
    fprintf('SB axis /9: UP=-1 Angle, MIDDLE=0 Angle, DOWN=+1 FA after UP qualification.\n');
    return;
end
hilite_system([ms '/FA Raw Buttons 1 to 32'],'find');
fprintf(['Disarmed: observe FA Raw Buttons 1 to 32 while toggling a FREE switch.\n' ...
    'Identify its actual button index; physical switch names do not imply indices.\n' ...
    'Expected encoded mode after mapping: switch OFF=0, ON=2.\n' ...
    'Mode 2 is a request, not proof of assist torque: calibration lock may still inhibit.\n']);
end
