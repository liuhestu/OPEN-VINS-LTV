"""Strict public live schema/causal-time checks; no reference accuracy claims."""
import argparse,json,math
from pathlib import Path

def check(directory):
    root=Path(directory)
    rows=[json.loads(s) for s in (root/'features.jsonl').read_text().splitlines()]
    meta=json.loads((root/'replay.json').read_text())
    assert len(rows)==meta['camera_packets']>0
    births=set();cold=raw=0;previous_ns=None
    for receipt,r in enumerate(rows,1):
        assert r.get('hardening') is True,'missing hardening schema'
        assert r['receipt_id']==receipt
        ns=r['camera_ns'];assert r['right_camera_ns']==ns
        assert previous_ns is None or ns>previous_ns;previous_ns=ns
        if r['raw_current']:
            raw+=1
            for name in ('v_body','eta_body'):
                assert isinstance(r[name],list) and len(r[name])==3 and all(math.isfinite(x) for x in r[name])
            assert r['observer_started'] and r['frame_matches_receipt']
            assert r['imu_time']==r['cursor']==r['target_imu_time']
        else:
            assert r['v_body'] is None and r['eta_body'] is None
            assert not r['ready_G'] and not r['ready_V']
        if r['ready_G'] or r['ready_V']:
            assert r['raw_current'] and r['observer_valid'] and r['pool_ready']
            assert 8<=math.sqrt(sum(x*x for x in r['eta_body']))<=11.5
        assert bool(r['joint_ready'])==bool(r['ready_G'] and r['ready_V'])
        if r['management_present']:
            assert r['frame_matches_receipt']
            assert len(r['retained_ids'])==len(set(r['retained_ids']))==r['state_features']
            for seed in r['seeds']:
                if seed['admitted']:
                    key=(r['epoch'],seed['id']);assert key not in births
                    births.add(key);assert seed['mean_written']
            assert r['admitted_tracks']==sum(epoch==r['epoch'] for epoch,_ in births)
            if not r['raw_current']:cold+=1
        if r['availability_state']=='COLLECTING' and r['reason'] not in ('uninitialized','waiting_clones','zupt'):
            assert r['management_present']
    return dict(status='PASS_ENGINEERING_ONLY',receipts=len(rows),raw_rows=raw,cold_management_rows=cold,births=len(births))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('directory');print(json.dumps(check(p.parse_args().directory),indent=2))
