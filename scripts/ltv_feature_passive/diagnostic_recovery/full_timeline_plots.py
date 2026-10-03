"""Presentation-only full physical-time axes, retaining unsupported GT gaps.

The frozen plotter uses Matplotlib autoscaling, which can hide leading/trailing
unsupported reference time. This wrapper sets the saved real timeline bounds,
without changing an array, metric, method, support mask or frozen source file.
"""
import argparse
import json
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import report_plots as original
from artifacts import sha


def build(manifest, out):
    old_real = original.real_figures
    old_save = original.save_figure
    timelines = {}
    def real(sequence, metrics, arrays, phase, destination, artifacts):
        limits = (0., float(arrays['time'][-1] - arrays['time'][0]))
        if limits[1] <= 0:
            raise ValueError('Empty real physical-time support')
        timelines[sequence] = limits
        def save(fig, path, output):
            for ax in fig.axes:
                ax.set_xlim(*limits)
            old_save(fig, path, output)
        original.save_figure = save
        try:
            old_real(sequence, metrics, arrays, phase, destination, artifacts)
        finally:
            original.save_figure = old_save
    original.real_figures = real
    try:
        proof = original.build(manifest, out)
    finally:
        original.real_figures = old_real
    proof['presentation_repair'] = {
        'wrapper': str(Path(__file__).resolve()), 'wrapper_sha256': sha(__file__),
        'real_full_elapsed_time_bounds_seconds': timelines,
        'description': 'Force full saved physical-time bounds; missing reference remains NaN and blank.',
        'numerical_arrays_or_metrics_modified': False}
    (out/'artifact_manifest.json').write_text(json.dumps(proof, indent=2)+'\n')
    return proof

if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest', required=True, type=Path)
    p.add_argument('--out', required=True, type=Path)
    a = p.parse_args()
    result = build(a.manifest, a.out)
    print(json.dumps({'status': 'PASS', 'figures': len(result['figures']),
                      'real_bounds': result['presentation_repair']['real_full_elapsed_time_bounds_seconds']}, indent=2))
