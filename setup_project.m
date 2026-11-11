function project=setup_project
% Set paths only. Does not load, build, download, start, or arm a model.
project=fileparts(mfilename('fullpath'));
addpath(project,fullfile(project,'optical_flow'), ...
    fullfile(project,'pid_diagnostics'),fullfile(project,'capture_tool'), ...
    fullfile(project,'qgc_bridge'));
end
