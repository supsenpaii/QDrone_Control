function [config,cameraToBody]=flow_assist_config
% Measured intrinsics, camera mounting and nominal optical depth are REQUIRED.
% Example orientation ONLY: top of image faces nose, camera looks down.
% Do not change confirmed to 1 until sign tests and measurements are complete.
cameraToBody=[0 -1 0;-1 0 0;0 0 -1];
confirmed=0;fx=0;fy=0;nominalOpticalDepth_m=0;
config=[confirmed;fx;fy;nominalOpticalDepth_m; ...
    0.03;3*pi/180;4*pi/180;0.10;0.35;0.10;0.35;1*pi/180;12*pi/180];
end
