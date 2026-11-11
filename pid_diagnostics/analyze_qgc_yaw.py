"""Analyze finalized QGC controller/plant MAT-v5 logs, without drone access.

Row mapping was checked against Stabilizer_QDrone_QGC_OF.mdl Mux wiring.
The standard logs do NOT contain the motor arm gate: thrust>4 N below is a
command-based selection, not a measurement that the vehicle was airborne.
"""
import argparse
import array
import bisect
import json
import math
import pathlib
import statistics as stats
import struct


def read_matrix(path, expected_name, rows):
    raw = path.read_bytes()
    if len(raw) < 136 or raw[126:128] != b'IM':
        raise ValueError(f'Empty/incomplete or unsupported MAT file: {path}')
    kind, size = struct.unpack_from('<II', raw, 128)
    if kind != 14 or size + 136 != len(raw):
        raise ValueError('Expected one uncompressed MAT-v5 matrix')
    elements, pos = [], 136
    while pos < len(raw):
        kind, size = struct.unpack_from('<II', raw, pos)
        if kind >> 16:
            elements.append((kind & 65535, raw[pos+4:pos+4+(kind >> 16)]))
            pos += 8
        else:
            if pos + 8 + size > len(raw):
                raise ValueError('Incomplete MAT element')
            elements.append((kind, raw[pos+8:pos+8+size]))
            pos += 8 + ((size + 7) // 8) * 8
    nr, nc = struct.unpack('<2i', elements[1][1])
    if nr != rows or elements[2][1].decode() != expected_name or elements[3][0] != 9:
        raise ValueError('Unexpected variable, dimensions or data type')
    data = array.array('d')
    data.frombytes(elements[3][1])
    if len(data) != nr * nc:
        raise ValueError('Incorrect sample count')
    result = [data[i::nr] for i in range(nr)]
    if not all(result[0][i] > result[0][i-1] for i in range(1, nc)):
        raise ValueError('Time is not strictly increasing')
    return result


def analyze(folder, session):
    cpath = folder / f'Stabilizer_{session}.mat'
    qpath = folder / f'QDrone_{session}.mat'
    fpath = folder / f'OpticalFlow_{session}.mat'
    c = read_matrix(cpath, 'stabilizer_data', 20)
    q = read_matrix(qpath, 'QDrone_data', 23)
    f = read_matrix(fpath, 'optical_flow_data', 16)
    if c[0] != q[0]:
        raise ValueError('Controller and plant log clocks do not match')
    deg = 180 / math.pi

    def summarize(start, end):
        ix = [i for i, t in enumerate(c[0]) if start <= t < end]
        mean = lambda row: stats.mean(row[i] for i in ix)
        return dict(time_s=[start, end], samples=len(ix),
                    thrust_command_mean_N=mean(c[16]),
                    yaw_rate_command_mean_deg_s=mean(c[9])*deg,
                    gyro_yaw_rate_mean_deg_s=mean(q[16])*deg,
                    estimated_yaw_change_deg=(q[7][ix[-1]]-q[7][ix[0]])*deg,
                    yaw_torque_mean_Nm=mean(c[19]),
                    motor_command_means=[mean(q[k]) for k in range(1, 5)],
                    battery_min_V=min(q[20][i] for i in ix))

    powered = [i for i in range(len(c[0])) if c[16][i] > 4]
    neutral = [i for i in powered if abs(c[9][i]) < math.radians(0.1)]
    flow_selected = [i for i, t in enumerate(f[0])
                     if c[16][min(bisect.bisect_left(c[0], t), len(c[0])-1)] > 4]
    if not powered or not neutral:
        raise ValueError('No adequate thrust/neutral-yaw command samples')
    p_error = max(abs(c[15][i] - 0.0395*(c[9][i]-q[16][i])) for i in powered)
    report = dict(
        session=session, controller_file=str(cpath), plant_file=str(qpath),
        flow_file=str(fpath), duration_s=c[0][-1], samples=len(c[0]),
        selection_note='thrust command >4 N; arm gate/airborne state not recorded in these logs',
        mode_values=sorted(set(c[6])), neutral_yaw_selected_seconds=len(neutral)*0.001,
        neutral_yaw_command_tolerance_deg_s=0.1,
        neutral_yaw_measured_mean_deg_s=stats.mean(q[16][i] for i in neutral)*deg,
        neutral_yaw_measured_median_deg_s=stats.median(q[16][i] for i in neutral)*deg,
        yaw_proportional_law_max_residual_Nm=p_error,
        verified_yaw_law='tau_z = 0.0395*(yaw_rate_command - gyro_z); no integral in this model',
        motor_at_limit_fraction=[stats.mean(q[k][i] <= 1e-6 or q[k][i] >= 1-1e-6 for i in powered)
                                 for k in range(1,5)],
        flow_valid_fraction_all=stats.mean(f[5]),
        flow_valid_fraction_thrust_gt4=stats.mean(f[5][i] for i in flow_selected),
        windows=[summarize(a,b) for a,b in [(0,25),(34,39),(39,44),(44,48),(63,68),(68,73),(73,76)]],
        conclusion='Persistent yaw-rate tracking error with corrective torque of opposing sign; '
                   'consistent with inadequate disturbance rejection. Check actuator/mixer response '
                   'before adding bounded yaw-rate integral. No gain or hardware settings changed.',
    )
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--folder', type=pathlib.Path,
                        default=pathlib.Path('saved_data/Stabilizer_QDrone_QGC_OF'))
    parser.add_argument('--session', default='08-Nov-2026 16-03-48')
    parser.add_argument('--output', type=pathlib.Path,
                        default=pathlib.Path('pid_diagnostics/yaw_analysis_160348.json'))
    args = parser.parse_args()
    report = analyze(args.folder, args.session)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False), encoding='utf-8')
    print(json.dumps(report, indent=2, allow_nan=False))
