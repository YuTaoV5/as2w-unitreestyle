import csv
import json
import math
import statistics
import sys
from pathlib import Path

session = Path(sys.argv[1])
slots = [3, 0, 9, 6, 4, 1, 10, 7, 5, 2, 11, 8, 13, 12, 15, 14]
signs = [-1] * 4 + [1] * 12
state_names = ['Passive', 'FixStand', 'FixedStand', 'PolicyHandoff',
               'PolicyWalk', 'Brake', 'ReturnStand', 'FixSquat', 'EmergencyDamping']

def read(name, prefix):
    with (session / name).open() as stream:
        return [row for row in csv.reader(stream) if row and row[0] == prefix]

def pitch(row):
    w, x, y, z = map(float, row[7:11])
    sine = 2 * (w*y-z*x) / (w*w+x*x+y*y+z*z)
    return math.degrees(math.asin(max(-1, min(1, sine))))

def error(observation, command, axis):
    actual = float(observation[14 + 6*slots[axis]])
    target = float(command[7 + 7*axis])
    return signs[axis] * (actual - target)

commands = read('debug_lowcmd.csv', 'DEBUG_LOWCMD')
by_tick = {int(r[2]): r for r in commands}
observations = {}
for row in read('low_capture_fast.log', 'LOW_SAMPLE'):
    if len(row) >= 224 and int(row[4]) in by_tick:
        observations[int(row[4])] = row

runs = []
for row in commands:
    state = int(row[3])
    if not runs or int(runs[-1][-1][3]) != state:
        runs.append([])
    runs[-1].append(row)

summary = []
for run in runs:
    state = int(run[0][3])
    if state not in (2, 3, 4, 5, 6):
        continue
    start, end = int(run[0][1]), int(run[-1][1])
    pairs = [(observations[int(c[2])], c) for c in run if int(c[2]) in observations]
    result = {'state': state_names[state], 'duration_s': (end-start)/1e9, 'frames': len(run),
              'aligned_count': len(pairs),
              'leg_kp_first_last': [float(run[0][9]), float(run[-1][9])],
              'leg_kd_first_last': [float(run[0][10]), float(run[-1][10])]}
    if pairs:
        tail = [(r,c) for r,c in pairs if int(c[1]) >= end-1e9]
        if not tail:
            tail = pairs
        pitches = [pitch(r) for r,c in pairs]
        result.update(pitch_minmax_deg=[min(pitches), max(pitches)],
                      last_1s_pitch_deg=statistics.mean(pitch(r) for r,c in tail),
                      last_1s_rear_calf_error_rad=[statistics.mean(error(r,c,j) for r,c in tail) for j in (10,11)],
                      last_1s_rear_hip_error_rad=[statistics.mean(error(r,c,j) for r,c in tail) for j in (2,3)],
                      wheel_abs_dq_peak=max(abs(float(r[15+6*slots[j]])) for r,c in pairs for j in range(12,16)))
    summary.append(result)

policy = read('debug_policy.csv','DEBUG_POLICY')
protection = [dict(v.split('=',1) for v in r[2:] if '=' in v)
              for r in read('low_protection.log','PROTECT')]
stand_runs = [run for run in runs if int(run[0][3]) == 2]
stand_delta = None
if len(stand_runs) >= 2:
    before, after = stand_runs[0][-1], stand_runs[-1][-1]
    stand_delta = max(abs(float(after[7+7*j])-float(before[7+7*j])) for j in range(12))
print(json.dumps({'phases':summary, 'policy_frames':len(policy),
                  'fixedstand_pre_post_max_target_delta_rad':stand_delta,
                  'policy_nonzero_command_frames':sum(any(float(x)!=0 for x in r[5:8]) for r in policy),
                  'protection_rows':len(protection),
                  'protection_flagged_rows':sum(bool(int(r.get('motorstate_or','0'),0) or int(r.get('warning_or','0'),0)) for r in protection)},
                 indent=2))
