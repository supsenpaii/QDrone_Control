function result = analyze_pid_diagnostics(controllerFile, motorFile)
% Summarize matched passive logs; proposes no gains and writes no model.
c = read_matrix(controllerFile, 'pid_controller', 22);
m = read_matrix(motorFile, 'pid_motors', 15);
assert(all(diff(c(1,:)) > 0) && all(diff(m(1,:)) > 0), 'Non-monotonic log time.');
armed = c(21,:) > 0.5;
motorArmed = m(15,:) > 0.5;
assert(any(armed) && any(motorArmed), 'No armed data in these logs.');
result.rollErrorDeg = median((c(2,armed)-c(5,armed))*180/pi);
result.pitchErrorDeg = median((c(3,armed)-c(6,armed))*180/pi);
result.motorAtLowerFraction = mean(m(6:9,motorArmed) <= 1e-6, 2);
result.motorAtUpperFraction = mean(m(6:9,motorArmed) >= 1-1e-6, 2);
result.medianMotorCommand = median(m(6:9,motorArmed), 2);
result.medianBatteryV = median(m(14,motorArmed));
fprintf('Median roll tracking error: %.3f deg; pitch: %.3f deg\n', result.rollErrorDeg, result.pitchErrorDeg);
fprintf('Battery median: %.3f V\n', result.medianBatteryV);
for k = 1:4
    fprintf('PWM channel %d: command median %.4f; at lower limit %.1f%%; at upper %.1f%%\n', ...
        k-1, result.medianMotorCommand(k), 100*result.motorAtLowerFraction(k), 100*result.motorAtUpperFraction(k));
end
figure('Name','QDrone passive PID diagnostics');
subplot(3,1,1);plot(c(1,:), c([2 5],:)'*180/pi);ylabel('Roll deg');legend('Command','Measured');grid on;
subplot(3,1,2);plot(c(1,:), c([3 6],:)'*180/pi);ylabel('Pitch deg');legend('Command','Measured');grid on;
subplot(3,1,3);plot(m(1,:), m(6:9,:)');ylabel('Motor command');xlabel('Model time s');legend('PWM0','PWM1','PWM2','PWM3');grid on;
fprintf('Summary includes all armed samples; inspect the plots for the specific throttle interval.\n');
end
function v = read_matrix(file, name, rows)
s = load(file, name); assert(isfield(s,name), 'Missing expected log variable.');
v = s.(name);
assert(isnumeric(v) && ismatrix(v) && ~isempty(v), 'Invalid or empty log.');
if size(v,1)~=rows && size(v,1)~=rows+1 && (size(v,2)==rows || size(v,2)==rows+1), v=v'; end
% QUARC versions may prepend their own timestamp; explicit Clock follows it.
if size(v,1)==rows+1
    assert(max(abs(v(1,:)-v(2,:))) < 0.01, 'Unexpected extra row; inspect log format.');
    v=v(2:end,:);
end
assert(size(v,1)==rows && all(isfinite(v(:))), 'Unexpected log width or non-finite data.');
end
