"""Read finalized RYPI logs; commands are not measured motor thrust/RPM."""
import json
import math
import pathlib
import statistics as st
from analyze_qgc_yaw import read_matrix

D = 180 / math.pi
ROOT = pathlib.Path(__file__).resolve().parents[1]

def summary(a, ix, dt):
    if not ix:
        return None
    mean = lambda row: st.mean(row[i] for i in ix)
    return {
        'seconds': len(ix)*dt,
        'roll_command_mean_deg': mean(a[56])*D,
        'roll_measured_mean_deg': mean(a[8])*D,
        'roll_measured_median_deg': st.median(a[8][i] for i in ix)*D,
        'roll_tracking_MAE_deg': st.mean(abs(a[56][i]-a[8][i])*D for i in ix),
        'pitch_command_mean_deg': mean(a[57])*D,
        'pitch_measured_mean_deg': mean(a[9])*D,
        'PD_mean_Nm': [mean(a[k]) for k in range(11,14)],
        'I_mean_Nm': [mean(a[k]) for k in range(14,17)],
        'limited_torque_mean_Nm': [mean(a[k]) for k in range(23,26)],
        'roll_I_max_abs_Nm': max(abs(a[14][i]) for i in ix),
        'roll_I_at_cap_fraction': st.mean(abs(a[14][i]) >= .999*a[46][i] for i in ix),
        'raw_motor_command_mean': [mean(a[k]) for k in range(52,56)],
        'motor_command_clipped_fraction': [st.mean(a[k][i] <= 0 or a[k][i] >= 1 for i in ix) for k in range(52,56)],
        'roll_AW_residual_nonzero_fraction': st.mean(abs(a[26][i]) > 1e-8 for i in ix),
    }

reports = []
for path in sorted((ROOT/'saved_data/Stabilizer_QDrone_QGC_OF_RYPI').glob('RollYawPI_*.mat')):
    try:
        a = read_matrix(path, 'roll_yaw_pi_data', 60)
        dt = st.median(a[0][i]-a[0][i-1] for i in range(1,min(len(a[0]),1001)))
        selected = [i for i in range(len(a[0])) if a[39][i]>.5 and a[40][i]>=4 and a[41][i]<.5]
        slow = [i for i in selected if max(abs(a[k][i]) for k in (5,6,7))<10/D
                and max(abs(a[k][i]) for k in (8,9))<10/D]
        neutral = [i for i in slow if abs(a[56][i])<.1/D]
        r = dict(file=path.name, duration_s=a[0][-1],
                 enable=[sorted(set(a[k])) for k in (42,43)],
                 selected=summary(a,selected,dt), low_rate=summary(a,slow,dt),
                 low_rate_neutral_roll=summary(a,neutral,dt))
        if '17-48-33' in path.name:
            r['windows_low_rate'] = [dict(start_s=t, result=summary(a,[i for i in slow if t<=a[0][i]<t+20],dt))
                                     for t in range(0,420,20)]
        reports.append(r)
    except (ValueError,IndexError) as exc:
        reports.append(dict(file=path.name, excluded_reason=str(exc)))
out = dict(selection='actual Arm>0.5, throttle>=4N, mode<0.5; not proof of free flight',
           low_rate_selection='add |gyro XYZ|<10deg/s and |estimated roll,pitch|<10deg; not proof of equilibrium',
           motor_note='raw normalized pre-clipping commands; no measured RPM or individual thrust',
           sessions=reports)
dest = ROOT/'pid_diagnostics/roll_motor_bias_analysis.json'
dest.write_text(json.dumps(out,indent=2,allow_nan=False),encoding='utf-8')
for r in reports:
    print(r['file'])
    for key in ('low_rate','low_rate_neutral_roll'):
        if r.get(key): print(key,json.dumps(r[key]))
print(dest)
