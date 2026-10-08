"""Final evidence checks; scientific no-benefit is a valid outcome."""
import hashlib,subprocess
from common import *
from run import identity

def verify():
    manifest=read(OUT/'experiment_manifest.json');runs=latest_runs();paths=read(OUT/'final_paths.json')
    assert set(paths)==set(SEQUENCES)
    assert all(r['status']=='complete' for r in runs.values()),'failed/running attempts require explicit disposition'
    assert all(r['sequence'] in SEQUENCES and r['sequence']!='MH_04_difficult' for r in runs.values())
    full=sum(not r['short'] for r in runs.values());short=sum(r['short'] for r in runs.values())
    assert full<=80 and short<=20
    frozen=manifest['frozen_identity'];assert identity()==frozen
    assert all(r['identity']==frozen for r in runs.values())
    formal=[]
    for seq,modes in paths.items():
        assert set(modes)==set(MODES)
        for mode,rid in modes.items():
            r=runs[rid];assert r['sequence']==seq and r['mode']==mode and not r['short']
            assert r['candidate']==manifest['selected_candidate'];formal.append(rid)
            p=Path(r['path']);m=read(p/'metrics.json');a=read(p/'engineering_audit.json')
            assert a['status']=='PASS' and a['checks']['complete_sensors']
            assert a['checks']['shadow_actual_max_relative_error']<=1e-9
            assert m['status']=='VALID','coverage failure must remain explicit, not shrink evaluation support'
        for f in ['trajectory.csv','audit.csv']:
            assert sha(Path(runs[modes['B']]['path'])/f)==sha(Path(runs[modes['P']]['path'])/f)
    assert len(set(formal))==45
    repeat=read(OUT/'repeat_summary.json');assert len(repeat)==3
    assert all(set(x)=={'B','G','V','GV'} for x in repeat.values())
    assert all(r['trajectory_exact'] and r['audit_exact'] for modes in repeat.values() for r in modes.values())
    profile=read(OUT/'cost_profile.json');assert len(profile)==2
    assert all(all(v['exact_to_uninstrumented'].values()) for modes in profile.values() for v in modes.values())
    tests={p.stem:read(p) for p in (OUT/'tests').glob('*.command.json')}
    assert len(tests)==9 and all(x['exit_code']==0 for x in tests.values())
    protected=['ov_core','ov_init','ov_msckf/src/state','ov_msckf/src/ltv/ltv_observer.cpp','ov_msckf/src/ltv/ltv_observer.h',
               'ov_msckf/src/ltv/ltv_types.h','ov_msckf/src/ltv/LtvAdapter.cpp','ov_msckf/src/ltv/LtvAdapter.h',
               'ov_msckf/src/update/UpdaterLTV.cpp','ov_msckf/src/update/UpdaterLTV.h','ov_msckf/src/update/UpdaterHelper.cpp',
               'ov_msckf/src/ros/ROS2Visualizer.cpp','docs/ltv_handover','docs/ltv/final_report.md']
    diff=subprocess.check_output(['git','diff',manifest['start_head'],'--',*protected],cwd=ROOT)
    assert not diff,'protected implementation or old results changed'
    unchanged_protocol={}
    for name,h in manifest['analysis_protocol_sha'].items():
        if name.endswith('value_study_protocol.md'):continue # Append-only provenance addenda, not numerical changes.
        unchanged_protocol[name]=sha(ROOT/name)==h
    assert all(unchanged_protocol.values())
    # The original protocol remains a byte-identical prefix, before appended source evidence.
    protocol=(ROOT/'docs/ltv/value_study_protocol.md').read_bytes();want=manifest['analysis_protocol_sha']['docs/ltv/value_study_protocol.md']
    prefix_match=any(hashlib.sha256(protocol[:i]).hexdigest()==want for i in range(len(protocol)+1) if i==len(protocol) or protocol[i:i+1]==b'\n')
    assert prefix_match,'original protocol prefix changed'
    release=read(OUT/'provenance/gt_release_verification.json')
    assert set(release)==set(DEV+VALIDATION) and all(x['match'] and x['local_sha']==x['remote_sha'] for x in release.values())
    evidence={'status':'PASS','full_replays':full,'short_replays':short,'formal_runs':len(formal),'repeat_runs':12,
              'profile_runs':4,'frozen_identity_unchanged':True,'all_run_engine_identities_match':True,
              'B_P_exact_sequences':9,'repeat_exact_runs':12,'protected_paths_unchanged':protected,
              'original_numeric_protocol_unchanged':unchanged_protocol,'original_protocol_prefix_preserved':prefix_match,
              'native_tests':tests,'gt_release_verification_sha':sha(OUT/'provenance/gt_release_verification.json'),
              'input_hash_recheck_log':str(OUT/'input_final_check.stdout.log'),
              'source_files':{str(p.relative_to(ROOT)):sha(p) for p in sorted((ROOT/'docs/experiments/tools/ltv_value').glob('*')) if p.is_file() and p.suffix in ['.py','.cpp']}}
    assert (OUT/'input_final_check.stdout.log').read_text().count('unchanged')==9
    write(OUT/'completion_audit.json',evidence)
    write(ROOT/'docs/ltv/evidence/value_study/completion_audit.json',evidence)
    print('PASS',full,'full',short,'short',45,'formal',12,'repeat',flush=True)
    return evidence
if __name__=='__main__':verify()
