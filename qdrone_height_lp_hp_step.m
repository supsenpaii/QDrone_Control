function y = qdrone_height_lp_hp_step(base,t,reset)
%#codegen
% Post-filter the 8-element calibrated Kalman output.
% y(1): Kalman + low-pass height [m]; y(3): its discrete slope [m/s].
% y(9): high-pass variation of RAW height [m], not absolute height.
% y(10): Kalman height before low-pass [m]. Other fields keep their meaning.
% Filters reset on unavailable data and reinitialize on valid recovery.
persistent lowHeight highVariation previousRaw lastT
if isempty(lastT)
    lowHeight = 0;
    highVariation = 0;
    previousRaw = 0;
    lastT = NaN;
end
y = [base;NaN;NaN];
if reset
    lastT = NaN;
end
if base(4)==0 || ~isfinite(base(1)) || ~isfinite(base(2)) || ~isfinite(t)
    lastT = NaN;
    y(1) = NaN; y(3) = NaN; y(4) = 0;
    return;
end

tauLP = 2.0; % Low-pass time constant [s], cutoff about 0.080 Hz.
tauHP = 2.0; % High-pass time constant [s], cutoff about 0.080 Hz.
if ~isfinite(lastT) || t<lastT || t-lastT>0.5
    lowHeight = base(1);
    highVariation = 0;
    speed = 0;
elseif t==lastT
    lastT = NaN;
    y(1) = NaN; y(3) = NaN; y(4) = 0; y(8) = 4;
    return;
else
    dt = t-lastT;
    alpha = dt/(tauLP+dt);
    previousLow = lowHeight;
    lowHeight = lowHeight+alpha*(base(1)-lowHeight);
    speed = (lowHeight-previousLow)/dt;
    beta = tauHP/(tauHP+dt);
    highVariation = beta*(highVariation+base(2)-previousRaw);
end
lastT = t;
previousRaw = base(2);
y(1) = lowHeight;
y(3) = speed;
y(9) = highVariation;
y(10) = base(1);
end
