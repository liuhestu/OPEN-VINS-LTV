#!/usr/bin/env python3
"""Build source unchanged in a fresh /tmp directory; never invoke a ROS replay."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET

OUT = Path(__file__).resolve().parents[1]
REPO = OUT.parents[1]
if (OUT/'validation/results.json').exists() or (OUT/'parity_fixture/expected_outputs.jsonl').exists():
    raise RuntimeError('audit outputs already exist; replay in a fresh temporary directory using parity_fixture/README.md')
BUILD = Path(tempfile.mkdtemp(prefix='ltv-handover-build-'))
RECORDS = []


def run(name, command, cwd=REPO, env=None, output=None):
    result = subprocess.run([str(x) for x in command], cwd=cwd, env=env,
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    (OUT/'validation'/f'{name}.stdout.log').write_text(result.stdout)
    (OUT/'validation'/f'{name}.stderr.log').write_text(result.stderr)
    if output is not None and result.returncode == 0:
        output.write_text(result.stdout)
    RECORDS.append(dict(name=name,command=[str(x) for x in command],cwd=str(cwd),
                        exit_code=result.returncode,stdout=f'validation/{name}.stdout.log',
                        stderr=f'validation/{name}.stderr.log'))
    (OUT/'validation/results.json').write_text(json.dumps(
        dict(build_directory=str(BUILD),checks=RECORDS),indent=2)+'\n')
    print(name, result.returncode, flush=True)
    if result.returncode:
        raise RuntimeError(result.stderr[-2000:] or result.stdout[-2000:])
    return result


flags = ['g++','-std=c++14','-O1','-DNDEBUG','-Wextra','-Wpedantic',
         '-I'+str(REPO/'vins/src'),'-I/usr/include/eigen3','-I/usr/local/include']
observer = REPO/'vins/src/ltv/ltv_observer.cpp'
run('compile_observer',flags+['-c',observer,'-o',BUILD/'observer.o'])
run('compile_fixture',flags+[OUT/'parity_fixture/harness.cpp',BUILD/'observer.o',
                           '-o',BUILD/'fixture'])
run('generate_fixture',[BUILD/'fixture',OUT/'parity_fixture/inputs.jsonl',
                       OUT/'parity_fixture/expected_matrices.jsonl'],
    output=OUT/'parity_fixture/expected_outputs.jsonl')
second = run('repeat_fixture',[BUILD/'fixture',OUT/'parity_fixture/inputs.jsonl',
                              BUILD/'repeat_matrices.jsonl'])
assert second.stdout == (OUT/'parity_fixture/expected_outputs.jsonl').read_text()
assert (BUILD/'repeat_matrices.jsonl').read_bytes() == (OUT/'parity_fixture/expected_matrices.jsonl').read_bytes()
run('compile_alias_probe',flags+[OUT/'validation/eigen_alias_probe.cpp','-o',BUILD/'alias_probe'])
run('alias_probe',[BUILD/'alias_probe'])

names = ['observer','quality_gate','velocity_quality_gate','velocity_oracle_gate',
         'snapshot','gravity_factor','velocity_factor']
cpp = [REPO/f'vins/test/test_ltv_{n}.cpp' for n in names]
sources = [REPO/f'vins/src/ltv/ltv_{n}.cpp' for n in
           ['csv_logger','quality_gate','velocity_quality_gate','velocity_oracle_gate']]
sources += [REPO/f'vins/src/factor/{n}.cpp' for n in
            ['ltv_gravity_factor','ltv_velocity_factor','pose_local_parameterization']]
# Existing Oracle tests use fixed /tmp paths. Refuse to touch anyone's existing files.
for name in ['ltv_velocity_oracle_mask.csv','ltv_velocity_oracle_exact.csv',
             'ltv_velocity_oracle_malformed.csv','no_such_velocity_oracle_mask.csv']:
    if (Path('/tmp')/name).exists():
        raise RuntimeError('existing Oracle test path: '+name)
run('compile_existing_cpp_tests',flags+cpp+sources+[BUILD/'observer.o',
     '-L/usr/local/lib','-Wl,-rpath,/usr/local/lib','-lceres','-lgtest_main','-lgtest',
     '-pthread','-o',BUILD/'existing_cpp_tests'])
run('existing_cpp_tests',[BUILD/'existing_cpp_tests',
    '--gtest_output=xml:'+str(OUT/'validation/existing_cpp_tests.xml')])

environment = dict(os.environ,PYTHONDONTWRITEBYTECODE='1')
# Small existing unit tests only; their replay entry points are not called.
py_names = ['test_stage7_tuning','test_run_stage6_joint','test_verify_snapshot_timestamps',
            'test_build_stage5_oracle_mask','test_stage5_cache','test_evaluate_euroc_final',
            'test_run_euroc_final_experiment','test_audit_stage6b_common_alignment',
            'test_diagnose_stage6c_mh01','test_analyze_euroc_asl_velocity','test_evaluate_vins_euroc']
for name in py_names:
    run(name,['python3','-B',REPO/f'vins/test/{name}.py','-v'],cwd=BUILD,env=environment)

inputs = [json.loads(x) for x in (OUT/'parity_fixture/inputs.jsonl').read_text().splitlines()]
outputs = [json.loads(x) for x in (OUT/'parity_fixture/expected_outputs.jsonl').read_text().splitlines()]
camera = [x for x in outputs if x['op']=='camera']
assert any(x['camera_substeps']>1 for x in camera)
assert any(x['valid'] for x in camera)
compact = next(o for i,o in zip(inputs,outputs) if i.get('case')=='delete_middle_slot20_compact_add40')
assert compact['slots']['10']==0 and compact['slots']['20']==-1
assert compact['slots']['30']==1 and compact['slots']['40']==2
assert compact['started']
run_reason = {i['case']:o['reset_reason'] for i,o in zip(inputs,outputs) if 'case' in i}
assert run_reason['accepted_imu_timestamp_difference']=='none'
assert run_reason['timestamp_mismatch_gap']=='timestamp_gap'
assert run_reason['duplicate_frame_backward']=='timestamp_backward'
assert run_reason['camera_reset_gap']=='timestamp_gap'
assert run_reason['max_imu_dt_rejected']=='timestamp_gap'
assert run_reason['zero_dt_rejected']=='invalid_input'
assert run_reason['substep_limit_resets_not_clamps']=='covariance_failure'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
manifest = dict(generated=True,source_observer_unchanged=True,random_seed=42,
                random_sampling_used=False,input_definition='deterministic analytical synthetic sensor signals; no GT',
                event_count=len(inputs),build_directory=str(BUILD),compiler=flags,
                generation_command=RECORDS[2]['command'],repeat_byte_identical=True,
                excluded_comparison_fields=['wall_time','update_time_ms'],
                covariance_layout='P is full nested row-major logical JSON array',
                source_sha256={str(p.relative_to(REPO)):sha(p) for p in
                    [observer,REPO/'vins/src/ltv/ltv_observer.h',REPO/'vins/src/ltv/ltv_types.h',
                     REPO/'vins/src/utility/utility.h']},
                artifact_sha256={p.name:sha(p) for p in (OUT/'parity_fixture').iterdir() if p.is_file()},
                boundary_reset_results=run_reason,cpp_test_count=int(ET.parse(OUT/'validation/existing_cpp_tests.xml').getroot().attrib['tests']))
(OUT/'parity_fixture/fixture_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
print('FIXTURE GENERATED; existing C++ tests:',manifest['cpp_test_count'],flush=True)
