function y = qdrone_baro_height_kalman(p,t,gyroNorm,accelNorm,reset)
%#codegen
% Relative barometric height [m] and vertical speed [m/s], upward positive.
% p: RAW pressure [Pa]; t: time [s]; gyroNorm [rad/s]; accelNorm [m/s^2].
% reset: one-sample pulse. Call once per new barometer sample (0.025 s).
% Requires capture_tool/qdrone_baro_height.m on the MATLAB path.
% y=[height;raw_height;speed;valid;reference_Pa;progress;noise_m;status].
persistent x P lastT
if isempty(lastT)
    x = zeros(2,1);
    P = eye(2);
    lastT = NaN;
end
% Existing validation and 20-second stationary pressure reference.
y = qdrone_baro_height(p,t,gyroNorm,accelNorm,reset);
if reset || (isfinite(lastT) && isfinite(t) && t < lastT)
    lastT = NaN;
end
if y(4) == 0
    y(1) = NaN;
    y(3) = NaN;
    lastT = NaN;
    return;
end
sigma = max(0.05,y(7)); % Measurement standard deviation [m].
R = sigma^2;
q = 0.25;              % Acceleration spectral density [m^2/s^3].
z = y(2);              % Raw height, before the existing low-pass filter.
if ~isfinite(lastT)
    x = [z;0];
    P = [R 0;0 1];
else
    dt = t-lastT;
    A = [1 dt;0 1];
    Q = q*[dt^3/3 dt^2/2;dt^2/2 dt];
    % Predict state [height; vertical speed].
    xp = A*x;
    Pp = A*P*A' + Q;
    % Correct using height measurement, H=[1 0].
    K = Pp(:,1)/(Pp(1,1)+R);
    x = xp + K*(z-xp(1));
    % Joseph covariance update.
    J = eye(2)-K*[1 0];
    P = J*Pp*J' + K*R*K';
    P = (P+P')/2;
end
lastT = t;
y(1) = x(1);
y(3) = x(2);
end
