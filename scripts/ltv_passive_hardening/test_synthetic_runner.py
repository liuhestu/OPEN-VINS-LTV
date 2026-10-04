"""Short input-only Adapter fixture; --library must be built through budget."""
import argparse
import json
from pathlib import Path
import tempfile
import numpy as np
import synthetic_inputs
import synthetic_runner


def check(library):
    evidence=[]
    with tempfile.TemporaryDirectory() as folder:
        root=Path(folder)
        for condition in ('STEREO','TIME_ERROR'):
            inputs=root/condition
            synthetic_inputs.generate(inputs,synthetic_inputs.ROOT/'config/ltv_euroc/kalibr_imucam_chain.yaml',
                                      'REGULAR',42,.5,condition,.2)
            # The actual estimator must run successfully with evaluator labels absent.
            (inputs/'labels.npz').rename(inputs/'labels_not_available_to_estimation.npz')
            for mode in ('P_PREV','P_NEW','P_NEW_WARMUP'):
                out=root/(condition+'_'+mode)
                result=synthetic_runner.run(inputs,out,library,mode)
                rows=[json.loads(line) for line in (out/'events.jsonl').read_text().splitlines()]
                assert result['full_input_consumed'] and len(rows)==5
                assert result['mode']==mode
                assert [r['receipt_id'] for r in rows]==list(range(1,6))
                assert all(r['raw_current'] or (r['raw_v'] is None and r['raw_eta'] is None) for r in rows)
                if condition=='TIME_ERROR':
                    assert any(r['rejected_async_observation_indices'] for r in rows)
                    with np.load(inputs/'inputs.npz') as data:
                        for k,row in enumerate(rows):
                            a,b=data['observation_offsets'][k:k+2]
                            expected=np.flatnonzero(data['actual_ns'][a:b]!=data['camera_ns'][k]).tolist()
                            assert row['rejected_async_observation_indices']==expected
                else:assert all(not r['rejected_async_observation_indices'] for r in rows)
                with np.load(out/'states.npz') as states:
                    assert len(states['t'])==41
                    assert not states['raw_current'][1::10].any()
                with np.load(out/'matrices.npz') as matrices:
                    assert len(matrices['t'])==10
                    for d,P in zip(matrices['dimension'],matrices['P']):
                        if d:assert np.isfinite(P[:d,:d]).all()
                evidence.append({'condition':condition,'mode':mode,'events':len(rows),'status':'PASS'})
    return evidence


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--library',required=True)
    print(json.dumps(check(parser.parse_args().library),indent=2))
