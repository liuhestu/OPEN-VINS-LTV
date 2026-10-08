"""Unit-separated projected prior/noise ratios from frozen GV innovation blocks."""
import argparse
import json
from pathlib import Path
import numpy as np
from analyze import BLOCKS, SEQUENCES, readlines, stats


def main():
    p = argparse.ArgumentParser()
    p.add_argument('evidence', type=Path)
    p.add_argument('out', type=Path)
    args = p.parse_args()
    manifest = json.loads((args.out/'read_manifest.json').read_text())
    results = {}
    for seq in SEQUENCES:
        run = args.evidence/'final_runs'/f'{seq}_GV_01'
        events = json.loads((args.out/(run.name+'_compact.json')).read_text())['events']
        prior_std = {k:[] for k in BLOCKS}
        projected = {b:[] for b in ('G', 'V')}
        for e, r in zip(events, readlines(run/'matrices.jsonl', manifest)):
            assert e['camera_ns'] == r['camera_ns']
            if e['submit_reason'] != 'joint_applied':
                continue
            P, S, R = [np.asarray(r[k]) for k in ('prior_P', 'joint_S', 'joint_R')]
            for name, block in BLOCKS.items():
                # RMS of marginal component standard deviations, one state block only.
                value = np.sqrt(np.mean(np.diag(P)[block]))
                prior_std[name].append(value*180/np.pi if name=='attitude_deg' else value)
            start = e['visual_rows']
            for b in ('G', 'V'):
                count = e[b+'_rows']
                if count:
                    projected[b].append(float(np.trace(S[start:start+count, start:start+count]-
                                                      R[start:start+count, start:start+count])/
                                              np.trace(R[start:start+count, start:start+count])))
                start += count
        results[seq] = dict(prior_component_std={k:stats(v) for k,v in prior_std.items()},
                            projected_prior_variance_to_raw_noise={b:stats(v) for b,v in projected.items()})
    for path in (args.evidence/'final_runs').glob('*_GV_01/matrices.jsonl'):
        old = json.loads((args.out/'read_manifest.json').read_text())[str(path.resolve())]
        assert manifest[str(path.resolve())] == old
    (args.out/'noise.json').write_text(json.dumps(results, indent=2)+'\n')
    print(json.dumps(results, indent=2))


if __name__ == '__main__':
    main()
