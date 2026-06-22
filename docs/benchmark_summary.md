# Speed Mode Benchmark Summary

All numbers below are from local simulator runs on speed map `31`. Because each run has variance, treat single runs as weak evidence and 5-run batches as the minimum useful signal.

| Batch / Candidate | Limit | Success | Avg progress | Best progress | Note |
| --- | ---: | ---: | ---: | ---: | --- |
| `five_try_baseline_20260622_003355` | 130s | 0/5 | 82.74 | 90.05 | Baseline before right-edge exit guard |
| `five_try_right_edge_exit_20260622_004821` | 130s | 0/5 | 90.38 | 93.55 | Best verified direction so far |

## Current Unverified Candidate

The committed `python/my_car.py` includes one extra candidate change after the best verified batch:

- At `lap_progress >= 88.0`, if a hard corner is detected and speed is already `<= 105`, brake is reduced to `0.3` instead of the normal stronger hard-corner brake.

Unit tests pass for this behavior, but a fresh 5-run simulator benchmark is still required.

## Known Bottlenecks

- Around 69-72% progress: right-edge/wall approach. Improved by `_apply_map31_right_edge_exit_guard`.
- Around 88-90% progress: final hard-corner / finish-side obstacle or post. Latest candidate targets this area.
- Obstacle routing still has variance; repeated batches matter more than one good lap.
