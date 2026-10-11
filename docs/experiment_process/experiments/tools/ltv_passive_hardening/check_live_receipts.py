"""Independent live-log completeness check, without recovering missing rows."""
import argparse,csv,json
from pathlib import Path

def check(directory):
    p=Path(directory);metadata=json.loads((p/'replay.json').read_text())
    rows=[json.loads(x) for x in (p/'features.jsonl').read_text().splitlines()]
    with (p/'audit.csv').open() as f:audit=list(csv.DictReader(f))
    assert len(rows)==metadata['camera_packets']==len(audit)
    births=set();overlap=0;not_representable=0
    for index,(row,other) in enumerate(zip(rows,audit),1):
        assert row['receipt_id']==index
        assert row['camera_ns']==int(other['camera_ns'])
        if round(row['camera_ns']*1e-9*1e9)!=row['camera_ns']:not_representable+=1
        if row['available']:
            assert row['management_present'],('missing management',index)
            for seed in row['seeds']:
                if seed['admitted']:
                    key=(row['epoch'],seed['id']);assert key not in births;births.add(key)
                    assert seed['mean_written']
            assert row['admitted_tracks']==sum(e==row['epoch'] for e,_ in births)
            assert len(set(row['retained_ids']))==row['state_features']
            overlap+=1
    assert not_representable>0,'test failed to exercise epoch-nanosecond roundtrip'
    return {'status':'PASS','camera_receipts':len(rows),'available_complete_rows':overlap,'unique_births':len(births),'roundtrip_mismatch_packets':not_representable,'overlay_used':False}
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('directory',type=Path);a=p.parse_args();print(json.dumps(check(a.directory),indent=2))
