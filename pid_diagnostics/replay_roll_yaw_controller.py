"""Open-loop numerical checks of the compiled RYPI core on recorded inputs.

Does not simulate the changed aircraft trajectory or certify stability.
Allocator feedback is reconstructed from logged OLD-controller commands.
"""
import json
import math
import pathlib
import re

from analyze_qgc_yaw import read_matrix
from test_ry_native import Core, CFG


def inverse4(matrix):
    a=[list(row)+[float(i==j) for j in range(4)] for i,row in enumerate(matrix)]
    for i in range(4):
        pivot=max(range(i,4),key=lambda r:abs(a[r][i]))
        a[i],a[pivot]=a[pivot],a[i]
        divisor=a[i][i]
        if abs(divisor)<1e-12:raise ValueError('Singular mixer')
        a[i]=[x/divisor for x in a[i]]
        for r in range(4):
            if r!=i:
                f=a[r][i];a[r]=[x-f*v for x,v in zip(a[r],a[i])]
    return [row[4:] for row in a]


def main():
    root=pathlib.Path(__file__).resolve().parent.parent
    folder=root/'saved_data/Stabilizer_QDrone_QGC_OF_YawI'
    files=[p for p in folder.glob('YawI_*.mat') if p.stat().st_size>136]
    skipped=[]
    for path in sorted(files,key=lambda p:p.stat().st_mtime,reverse=True):
        y=read_matrix(path,'yaw_i_data',32)
        if any(a>.5 and t>=4 for a,t in zip(y[23],y[24])):break
        skipped.append(dict(file=str(path),samples=len(y[0]),reason='No actual Arm/throttle>=4N samples'))
    else:raise ValueError('No finalized flight/trial log has Arm/throttle data')
    generated_folder = 'Stabilizer_QDrone_QGC_OF_YawI_quarc_linux_x64'
    generated_file = 'Stabilizer_QDrone_QGC_OF_YawI_data.c'
    data_path = root / generated_folder / generated_file
    if not data_path.is_file():
        data_path = root / '_archive' / '20261109_cleanup' / 'build' / generated_folder / generated_file
    data = data_path.read_text()
    match=re.search(r"Referenced by: '<S2>/Constant1'\s*\*/\s*\{([^}]+)\}",data)
    if not match:raise ValueError('Cannot locate the actual compiled mixer')
    flat=[float(x.strip()) for x in match[1].split(',')]
    if len(flat)!=16:raise ValueError('Unexpected mixer dimensions')
    mixer=[[flat[r+4*c] for c in range(4)] for r in range(4)]
    inverse=inverse4(mixer)
    error=max(abs(sum(mixer[r][k]*inverse[k][c] for k in range(4))-float(r==c))
              for r in range(4) for c in range(4))
    assert error<1e-10
    inv_col=[inverse[r][c] for c in range(4) for r in range(4)]
    core=Core();previous_residual=[0.,0.,0.];previous=[0.,0.,0.]
    counts={str(i):[0,0] for i in range(7)}
    selected=old_resets=new_zero_events=retained=0
    cap_counts=[0,0];max_abs=[0.,0.];dt=.002
    for i in range(len(y[0])):
        if i and abs((y[0][i]-y[0][i-1])-.002)>1e-7:
            # Do not bridge a discontinuity as though continuous integration occurred.
            core.state[:]=[0,0,0,0];previous_residual=[0,0,0];previous=[0,0,0]
        e=[y[k][i] for k in (2,3,4)]
        angles=[y[k][i] for k in (8,9,10)]
        gyro=[y[k][i] for k in (5,6,7)]
        raw=[y[k][i] for k in (17,18,19)]
        limited=[y[k][i] for k in (20,21,22)]
        force=[y[24][i]]+limited
        motor=[sum(mixer[r][k]*force[k] for k in range(4)) for r in range(4)]
        residual=core.aw(motor,raw,limited,inv_col)
        out,diag=core.step(error=e,angles=angles,gyro=gyro,residual=previous_residual,
                           arm=y[23][i],throttle=y[24][i],mode=y[25][i],dt=dt)
        previous_residual=residual
        assert out[1]==0 and all(math.isfinite(x) for x in out)
        assert abs(out[0])<=CFG[2]+1e-12 and abs(out[2])<=CFG[3]+1e-12
        if y[23][i]<.5 or y[24][i]<4:assert out==[0,0,0]
        for k,axis in enumerate((0,2)):max_abs[k]=max(max_abs[k],abs(out[axis]))
        if y[23][i]>.5 and y[24][i]>=4:
            selected+=1
            for k in range(2):
                counts[str(int(diag[k]))][k]+=1
                cap_counts[k]+=abs(out[(0,2)[k]])>=CFG[2+k]-1e-5
            if i and y[27][i]<.5 and y[27][i-1]>.5 and abs(y[16][i-1])>1e-5:old_resets+=1
            if abs(previous[2])>1e-5 and out[2]==0 and int(diag[1])==5:new_zero_events+=1
            if int(diag[1])==2 and abs(out[2])>1e-5:retained+=1
        previous=out
    result=dict(source_log=str(path),newer_logs_without_powered_samples=skipped,samples=len(y[0]),log_dt_s=dt,
        selected_actual_arm_throttle_samples=selected,config=CFG,
        old_logged_active_to_inactive_with_I=old_resets,
        replay_new_hard_reset_to_zero_yaw_events=new_zero_events,
        replay_transient_pause_with_retained_yaw_I_samples=retained,
        replay_status_counts_roll_yaw=counts,
        replay_at_cap_fraction_roll_yaw=[x/selected for x in cap_counts],
        replay_max_abs_I_roll_yaw=max_abs,mixer_inverse_max_error=error,
        checks='PASS: finite, bounded, pitch untouched, reset when disarmed/low throttle',
        limitations='Open-loop replay on the OLD controller trajectory, 2 ms logging vs 1 ms target. '
          'Allocator feedback reconstructed from OLD commands with a one-log-sample delay. '
          'NOT a prediction of new yaw/roll response, NOT flight or hardware validation.')
    output=root/'pid_diagnostics/roll_yaw_replay_latest.json'
    output.write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
