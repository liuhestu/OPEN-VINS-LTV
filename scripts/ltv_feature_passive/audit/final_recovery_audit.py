"""Independent artifact-only audit; no observer execution or GT access."""
import copy
import hashlib
import itertools
import json
from pathlib import Path

OUT = Path('/home/he/output/ltv_feature_readiness_passive')
DOC = Path('docs/ltv/feature_readiness_passive/drafts')
MANAGEMENT = {'retained_ids','opportunity_tracks','admitted_tracks','retired_ids','seeds','tracks'}

def load(path):
    return json.loads(Path(path).read_text())

def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for chunk in iter(lambda:f.read(1024*1024),b''):
            h.update(chunk)
    return h.hexdigest()

def strip(value):
    value=copy.deepcopy(value)
    value.pop('diagnostic_recovery',None)
    value.pop('feature_management')
    value['tables'].pop('seed_reliability_available_evidence')
    value['tables']['feature_management'].pop('management')
    return value

def main():
    original=load(OUT/'final_manifest.json')
    revised=load(OUT/'final_manifest_recovered.json')
    results=[]
    for seq, oldpath in original['real_metrics'].items():
        p=Path(revised['real_metrics'][seq]);new=load(p);old=load(oldpath)
        assert strip(old)==strip(new), f'{seq}: scientific metrics changed'
        root=p.parent.parent
        binding=load(root/'binding.json');proof=load(root/'exact_report.json')
        audit=load(p.parent/'recovery_audit.json')
        build=load(binding['buildproof'])
        assert sha(binding['buildproof'])==binding['buildproof_sha256']
        assert build['binary_sha256']==binding['binary_sha']
        assert build['freeze_sha256']==binding['freeze_sha']==original['freeze_sha']
        assert build['runtime']==binding['runtime']==old['seed_source_identity']['runtime']
        assert sha(root/'exact_report.json')==binding['report_sha256']
        assert sha(root/'diagnostics.jsonl')==binding['recovered_sha256']
        assert binding['attempt']['exit_code']==0 and proof['status']=='PASS'
        assert proof['comparison']=='exact_frozen_cache_non_wallclock'
        assert sha(p.parent/'error_curves.npz')==sha(Path(oldpath).parent/'error_curves.npz')
        for path, expected in audit['sha'].items():
            assert sha(path)==expected, (seq,path)
        assert audit['binding']==binding and audit['GT_reread'] is False
        packets=0
        with (Path(audit['original_run'])/'features.jsonl').open() as left, (p.parent/'overlay_features.jsonl').open() as right:
            for x,y in itertools.zip_longest(left,right):
                assert x is not None and y is not None
                x,y=json.loads(x),json.loads(y)
                assert {k:v for k,v in x.items() if k not in MANAGEMENT}=={k:v for k,v in y.items() if k not in MANAGEMENT}
                packets+=1
        admitted=set();ttl=set();retired=set();sources={};events=0
        with (root/'diagnostics.jsonl').open() as f:
            for line in f:
                r=json.loads(line);events+=1;ep=r['epoch']
                for s in r['seeds']:
                    if s.get('admitted'):
                        key=(ep,s['id']);assert key not in admitted
                        admitted.add(key);sources[s['source']]=sources.get(s['source'],0)+1
                ttl.update((ep,i) for i in r['never_admitted_candidate_ttl_ids'])
                retired.update((ep,i) for i in r['admitted_retired_ids'])
                if r['event_type']=='process' and r['available'] and r['management_accepted_input']:
                    assert len({i for e,i in admitted if e==ep})==r['admitted_tracks']
                    assert len(set(r['retained_ids']))==len(r['retained_ids'])==r['state_features']
        assert not ttl&admitted and retired<=admitted
        m=new['feature_management']
        assert len(admitted)==m['admitted_tracks']
        assert len(ttl)==m['never_admitted_candidate_ttl_tracks']
        assert len(retired)==m['admitted_active_removal_tracks']
        assert sources==m['source_admissions'], (sources,m['source_admissions'])
        assert events==proof['compared_events']==len(audit['matches'])
        results.append(dict(sequence=seq,status='PASS',packets=packets,events=events,admitted=len(admitted),candidate_ttl=len(ttl),admitted_removals=len(retired),replay_attempt=binding['attempt']['id'],cache_sha256=binding['cache_sha256']))
    assert len(results)==6
    repeats=[]
    for seq,path in original['repeat_checks'].items():
        r=load(path);item=next(x for x in results if x['sequence']==seq)
        assert r['status']=='PASS' and r['repeated_auxiliary_exact']
        assert r['auxiliary_sha256']['cache.bin']==item['cache_sha256']
        repeats.append(dict(sequence=seq,status='IDENTICAL_INPUT_INFERENCE_ONLY',basis=path,additional_recovery_repeat_executed=False))
    report=dict(status='PASS',sequences=results,repeats=repeats,scientific_metrics_exact=True,nonmanagement_packets_exact=True,GT_accessed=False,cache_bytes_rehashed=False,cache_proof='Exact replay and bound overlay validation already checked original cache bytes; independent audit crosschecks recorded cache SHA with prescribed repeat evidence.',manifest_sha256=sha(OUT/'final_manifest_recovered.json'))
    (DOC/'audit_final_recovery.json').write_text(json.dumps(report,indent=2)+'\n')
    (DOC/'audit_final_recovery.md').write_text('# Independent final recovery audit\n\nAll six recovered metrics preserve every non-management scientific metric exactly. Original and overlay packets have identical non-management fields and packet counts. Error curves are byte-identical. Recovery reports, build identities, recorded SHA bindings, complete event counts, unique admissions, seed-source counts and separated TTL/active-removal counts agree. No GT was accessed and no observer was executed by this audit.\n\nThe two prescribed repeats have recorded byte-identical caches whose SHA matches the primary recovery cache binding. Equivalent recovered diagnostics are an inference from identical input and deterministic implementation, not additional executed recovery repeats. Large cache files were not redundantly rehashed here; exact replays and overlay binding checks performed that verification.\n\nFrozen live capture retains its documented logging defect. Completed evidence is the frozen capture plus mandatory exact-cache recovery and verified overlay; an unapplied patch alone is not a repaired live logger.\n')
    print(json.dumps(report))

if __name__=='__main__':main()
