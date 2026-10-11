"""Plot every saved sample in the two user-selected fixed neighborhoods."""
import argparse
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def run(source, out):
    source, out = Path(source), Path(out)
    data = json.loads(source.read_text())
    out.mkdir(parents=True, exist_ok=False)
    fig, axes = plt.subplots(3, len(data['events']), figsize=(12, 9), squeeze=False)
    receipt = []
    for column, event in enumerate(data['events']):
        rows = event['neighborhood']
        times = np.array([r['relative_initialized_s'] for r in rows])
        target = event['target']['relative_initialized_s']
        def values(quantity, field):
            return np.array([r['quantities'][quantity].get(field, np.nan)
                             if r['quantities'][quantity]['reference_valid'] else np.nan
                             for r in rows])
        ax = axes[0, column]
        for field, label, style in [('pre_error_norm', 'LTV pre-correction', '--'),
                                    ('post_error_norm', 'LTV post-correction', '-'),
                                    ('OpenVINS_error_norm', 'OpenVINS', ':')]:
            ax.plot(times, values('v', field), style, marker='.', label=label)
        ax.set_title(f'init + {target:.2f} s')
        ax.set_ylabel('Velocity error norm [m/s]')
        ax.legend(fontsize=8)
        ax = axes[1, column]
        pre, post = values('v', 'pre_error_vector'), values('v', 'post_error_vector')
        for component, color in enumerate(['tab:red', 'tab:green', 'tab:blue']):
            ax.plot(times, pre[:, component], '--', color=color, alpha=.6)
            ax.plot(times, post[:, component], '.-', color=color,
                    label=f'{"xyz"[component]} post (dashed: pre)')
        ax.axhline(0, color='gray', linewidth=.5)
        ax.set_ylabel('Body velocity error components [m/s]')
        ax.legend(fontsize=8)
        ax = axes[2, column]
        ax.plot(times, values('eta', 'pre_error_norm'), '.--', label='eta pre')
        ax.plot(times, values('eta', 'post_error_norm'), '.-', label='eta post')
        ax.set_ylabel('Gravity vector error norm [m/s²]')
        ax.set_xlabel('Time after initialization [s]')
        ax.legend(fontsize=8)
        for ax in axes[:, column]:
            ax.axvline(target, color='black', linewidth=.8)
            ax.grid(alpha=.2)
        receipt.append({'target_camera_ns': event['target_camera_ns'],
                        'samples': len(rows), 'first': float(times[0]), 'last': float(times[-1]),
                        'ready_G_samples': sum(bool(r['ready_G']) for r in rows),
                        'ready_V_samples': sum(bool(r['ready_V']) for r in rows)})
    fig.suptitle('Same-time reference before and after actual LTV camera correction\n'
                 'Fixed user-selected neighborhoods; all samples retained; no ready mask')
    fig.tight_layout(rect=(0, 0, 1, .94))
    for suffix in ['png', 'pdf']:
        fig.savefig(out / f'local_correction_error.{suffix}', dpi=160)
    plt.close(fig)
    receipt = {'source': str(source), 'sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
               'neighborhoods': receipt, 'selection': 'All saved rows; no ready/error filtering'}
    (out / 'receipt.json').write_text(json.dumps(receipt, indent=2) + '\n')
    print(json.dumps(receipt, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--source', required=True)
    parser.add_argument('--out', required=True)
    run(**vars(parser.parse_args()))
