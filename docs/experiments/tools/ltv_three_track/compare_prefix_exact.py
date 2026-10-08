#!/usr/bin/env python3
"""Exact paired prefix contract; never substitutes for full replay completion."""
import argparse
import json
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ltv_integration_cleanup'))
from check_real_outputs import csv_exact,jsonl_exact,cache_exact,first_difference,sha

p=argparse.ArgumentParser();p.add_argument('before',type=Path);p.add_argument('after',type=Path);p.add_argument('output',type=Path);p.add_argument('--seconds',type=float,required=True);a=p.parse_args()
if a.output.exists():raise RuntimeError('refusing comparison overwrite')
result={'scope':'matched fixed prefix only; full completion NOT_PROVEN','tolerance':0,'seconds':a.seconds,'excluded_fields':['features.compute_time_ms']}
try:
 result['audit_rows']=csv_exact(a.before/'audit.csv',a.after/'audit.csv');result['trajectory_rows']=csv_exact(a.before/'trajectory.csv',a.after/'trajectory.csv');result['unmatched_rows']=csv_exact(a.before/'unmatched_camera.csv',a.after/'unmatched_camera.csv')
 result['features']=jsonl_exact(a.before/'features.jsonl',a.after/'features.jsonl');result['cache']=cache_exact(a.before/'cache.bin',a.after/'cache.bin')
 for name in ['effective_options.json','replay.json']:
  difference=first_difference(json.loads((a.before/name).read_text()),json.loads((a.after/name).read_text()))
  if difference:raise AssertionError({'file':name,'difference':difference})
 metadata=json.loads((a.before/'replay.json').read_text());options=json.loads((a.before/'effective_options.json').read_text())
 assert metadata['short_limit_seconds']==a.seconds and a.seconds>0 and not metadata['complete']
 assert metadata['camera_packets']>0 and metadata['imu_consumed']>0 and metadata['output_rows']>0
 assert result['audit_rows']==metadata['camera_packets']==result['features']['rows']
 assert result['trajectory_rows']==metadata['output_rows']
 assert result['cache']['counts']['process']+result['cache']['counts']['pause']==metadata['camera_packets']
 assert result['cache']['counts']['imu']==metadata['imu_consumed']
 assert not any(metadata[k] for k in ['actual_G_submissions','actual_V_submissions'])
 assert not any(options[k] for k in ['ltv_enable_gravity','ltv_enable_velocity'])
 result['status']='PASS_EXACT_PREFIX';result['input_consumption']=metadata
 result['files']={name:{'before_sha':sha(a.before/name),'after_sha':sha(a.after/name)} for name in ['audit.csv','trajectory.csv','cache.bin','features.jsonl','effective_options.json','replay.json','unmatched_camera.csv']}
except Exception as e:
 result['status']='FAIL_EXACT_PREFIX';result['error']=str(e)
a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result));raise SystemExit(result['status']!='PASS_EXACT_PREFIX')
