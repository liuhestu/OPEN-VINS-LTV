"""Exactly two V20 parity replays followed by two V10 full-input replays."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'scripts/ltv_gv_evaluation'))
from run_matrix import run, write, budget, exact


def parity(old, new):
    result = dict(status='PASS_EXACT_V20', tolerance=0, excluded=['features.jsonl.compute_time_ms'],
                  instrumentation_difference='new velocity_confirm_frames metadata = 20; algorithm option disabled')
    for name in ('audit.csv', 'trajectory.csv', 'ltv.csv', 'unmatched_camera.csv'):
        result[name] = exact.csv_exact(old/name, new/name)
    result['features.jsonl'] = exact.jsonl_exact(old/'features.jsonl', new/'features.jsonl')
    result['cache.bin'] = exact.cache_exact(old/'cache.bin', new/'cache.bin')
    for name in ('fusion.jsonl', 'matrices.jsonl', 'effective_options.json', 'replay.json'):
        assert exact.sha(old/name) == exact.sha(new/name), name
        result[name] = exact.sha(new/name)
    a, b = [json.loads((p/'instrumentation.json').read_text()) for p in (old,new)]
    assert b.pop('velocity_confirm_frames') == 20
    assert a == b
    return result


def main():
    p = argparse.ArgumentParser()
    p.add_argument('out', type=Path)
    args = p.parse_args()
    out = args.out.resolve()
    frozen = json.loads((out/'freeze.json').read_text())
    write(out/'budget_before.json', budget.usage())
    status = {}
    with ThreadPoolExecutor(max_workers=2) as pool:
        for mode in ('V','V10'):
            futures = {s:pool.submit(run, out, s, mode) for s in frozen['inputs']}
            for seq, future in futures.items():
                assert future.result(), 'Replay failed: preserve its evidence and stop'
                status[seq+'/'+mode] = 'COMPLETE'
                if mode == 'V':
                    comparison = parity(Path(frozen['original_V_runs'][seq]), out/'final_runs'/f'{seq}_V_01')
                    write(out/f'parity_{seq}_V20.json', comparison)
                    status[seq+'/parity'] = 'PASS_EXACT'
                write(out/'matrix_status.json', status)
    old = Path(next(iter(frozen['original_V_runs'].values()))).parents[1]
    # Fixed OFF support is reused, not rerun. Explicit frozen external pointers.
    for seq in frozen['inputs']:
        (out/'final_runs'/f'{seq}_OFF_01').symlink_to(old/'final_runs'/f'{seq}_OFF_01')
    write(out/'budget_after.json', budget.usage())
    assert len(status) == 6 and budget.usage()['real']-json.loads((out/'budget_before.json').read_text())['real'] == 4


if __name__ == '__main__':
    main()
