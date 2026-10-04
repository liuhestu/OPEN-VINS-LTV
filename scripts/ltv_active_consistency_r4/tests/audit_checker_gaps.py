"""Fabricated read-only counterexamples to current checker; no sensor/replay inputs."""
import importlib.util,json,tempfile
from pathlib import Path
src=Path(__file__).resolve().parents[1]/'check_live_log.py'
spec=importlib.util.spec_from_file_location('r4_checker_subject',src);mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
def check(rows):
 with tempfile.TemporaryDirectory() as d:
  p=Path(d)/'features.jsonl';p.write_text(''.join(json.dumps(r)+'\n' for r in rows));return mod.check(p)
def row(action='SKIP',count=1):
 d={'feature_id':17,'timestamp':1.,'evaluable':True,'consistency_pass':False,'consistency_fail_count':count,'consistency_reject_count':999,'reason':'HOLDOUT_INCONSISTENT','history_max_residual_rad':.01,'action':action}
 return {'management_present':True,'active_consistency_schema':'ACTIVE_CONSISTENCY_V1','consistency_active_count':1,'consistency_evaluable_count':1,'consistency_pass_count':0,'consistency_skip_count':int(action=='SKIP'),'consistency_retire_count':int(action=='RETIRE'),'active_consistency':[d],'imu_time':1.,'epoch':1,'retained_ids':[17] if action=='SKIP' else [],'observations_sent_to_ltv':1,'corrected_ids':[17],'retirement_events':[]}
a=check([]);b=check([row()]);first=row();second=row('RETIRE',2);second['imu_time']=2;second['active_consistency'][0]['timestamp']=2;c=check([first,second])
assert all(x['status']=='PASS_LIVE_COUNTERS_AND_LIFECYCLE' for x in (a,b,c))
print(json.dumps({'status':'CONFIRMED_CHECKER_GAPS_NOT_ENGINEERING_PASS','empty_file_accepted':True,'rejected_id_still_corrected_accepted':True,'reject_counter_999_accepted':True,'retire_without_typed_event_accepted':True,'scope':'Fabricated small rows only; no runner executions or scientific thresholds changed.'},indent=2))
