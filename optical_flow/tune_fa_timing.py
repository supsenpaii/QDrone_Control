"""Estimate image/gyro timing from disarmed logs, with separate validation logs.

This fits rotational image motion with the existing camera geometry. Translation,
mounting error and rolling/exposure timing can confound the estimate. The output
is an offline candidate and does not change the model or flight-verification gate.
"""
import bisect
import json
import math
import pathlib
import sys

PROJECT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT / 'pid_diagnostics'))
from analyze_qgc_yaw import read_matrix


def load_session(stamp):
    folder = PROJECT / 'saved_data' / 'Stabilizer_QDrone_QGC_OF_RYPI_FA'
    paths = list(folder.glob(f'FlowAssist_* {stamp}.mat'))
    if len(paths) != 1:
        raise ValueError(f'Expected one session for {stamp}')
    data = read_matrix(paths[0], 'flow_assist_data', 66)
    if any(data[23]) or set(data[43]) != {2.0}:
        raise ValueError('Require an entirely disarmed shadow session')
    # Timing analysis requires one unchanged camera configuration.
    if any(min(row) != max(row) for row in data[43:65]):
        raise ValueError('Configuration changed within session')
    time = data[0]
    integral = [[0.0] * len(time) for _ in range(3)]
    for i in range(1, len(time)):
        dt = time[i] - time[i - 1]
        for axis in range(3):
            integral[axis][i] = integral[axis][i - 1] + dt * (
                data[17 + axis][i - 1] + data[17 + axis][i]) / 2

    def cumulative(axis, t):
        i = bisect.bisect_right(time, t) - 1
        if i < 0 or i >= len(time) - 1:
            raise ValueError('Interpolation outside logged interval')
        dt = t - time[i]
        slope = (data[17 + axis][i + 1] - data[17 + axis][i]) / (time[i + 1] - time[i])
        return integral[axis][i] + data[17 + axis][i] * dt + slope * dt * dt / 2

    frames = []
    previous = None
    for i, frame_time in enumerate(data[8]):
        if frame_time == previous:
            continue
        previous = frame_time
        dt = data[13][i]
        if (data[26][i] < .5 or data[12][i] < .5 or data[11][i] < .35
                or not .015 <= dt <= .08
                or frame_time - dt - .1 <= time[0] or frame_time + .1 >= time[-1]):
            continue
        # Select once at zero lag so every timing candidate uses identical frames.
        gyro = [(cumulative(k, frame_time) - cumulative(k, frame_time - dt)) / dt
                for k in range(3)]
        if max(abs(value) for value in gyro) >= .03:
            frames.append((i, frame_time, dt))
    if len(frames) < 100:
        raise ValueError('Insufficient rotational excitation for timing estimation')
    return paths[0], data, frames, cumulative


def score(session, lag_ms):
    _, data, frames, cumulative = session
    rotation = [row[0] for row in data[56:65]]
    fx, fy, height = (data[k][0] for k in (44, 45, 46))
    squared_error = predicted_square = observed_square = cross = 0.0
    for i, frame_time, dt in frames:
        end = frame_time + lag_ms / 1000
        gyro = [(cumulative(k, end) - cumulative(k, end - dt)) / dt for k in range(3)]
        camera_gyro = [sum(rotation[k + 3 * axis] * gyro[k] for k in range(3))
                       for axis in range(3)]
        lever = [-.03 * gyro[1], .1 * gyro[2] + .03 * gyro[0], -.1 * gyro[1]]
        camera_lever = [sum(rotation[k + 3 * axis] * lever[k] for k in range(3))
                        for axis in range(3)]
        predicted = [-camera_gyro[1] - camera_lever[0] / height,
                     camera_gyro[0] - camera_lever[1] / height]
        observed = [data[9][i] / (fx * dt), data[10][i] / (fy * dt)]
        for x, y in zip(predicted, observed):
            squared_error += (y - x) ** 2
            predicted_square += x * x
            observed_square += y * y
            cross += x * y
    return dict(frames=len(frames), rmse_per_s=math.sqrt(squared_error / (2 * len(frames))),
                gain_diagnostic=cross / predicted_square,
                correlation=cross / math.sqrt(predicted_square * observed_square))


def main():
    training = [load_session(stamp) for stamp in ('17-50-23', '17-59-48')]
    validation = [load_session(stamp) for stamp in ('18-16-27', '18-41-11')]
    candidates = range(-100, 101)
    lag = min(candidates, key=lambda value: sum(score(s, value)['rmse_per_s'] ** 2
                                               for s in training))
    results = []
    for role, sessions in (('training', training), ('validation', validation)):
        for session in sessions:
            baseline = score(session, 0)
            candidate = score(session, lag)
            results.append(dict(file=str(session[0]), role=role, baseline=baseline,
                                candidate=candidate, improvement_percent=100 * (
                                    1 - candidate['rmse_per_s'] / baseline['rmse_per_s'])))
    report = dict(candidate_gyro_window_shift_relative_frame_timestamp_ms=lag,
                  compensation_gain=1.0, sessions=results,
                  validation_improved=all(r['improvement_percent'] > 0 for r in results
                                          if r['role'] == 'validation'),
                  model_changed=False, flight_validated=False,
                  limitation='Hand motion can include translation. Optical exposure timestamps '
                             'and pure-yaw compensation remain unverified.')
    output = PROJECT / 'optical_flow' / 'fa_timing_candidate.json'
    output.write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
