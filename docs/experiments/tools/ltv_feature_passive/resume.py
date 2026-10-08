"""Inspect/reconcile interrupted attempts without refunding budgets or restarting jobs."""
import argparse
import json
import time
import budget

def resume(reconcile=False,out=budget.OUT):
    with budget.locked(out):
        records={}
        for event in budget.events(out):records.setdefault(event['id'],{})[event['event']]=event
        pending=[]
        for run_id,events in records.items():
            reserved=events['reserved']
            if reserved['kind']=='candidate' or 'finished' in events:continue
            started=events.get('started')
            if not started:
                status='RESERVED_WITHOUT_START_REQUIRES_REVIEW'
            elif started.get('owner') and budget.process_alive(started['owner']):
                status='LIVE_OWNER'
            elif started.get('process') and budget.process_alive(started['process']):
                status='LIVE_CHILD'
            elif not started.get('process') and budget.process_identity(started['pid']):
                status='LEGACY_PID_PRESENT_REQUIRES_REVIEW'
            else:
                status='INTERRUPTED'
                if reconcile:
                    budget.append({'event':'finished','id':run_id,'exit_code':None,
                        'error':'INTERRUPTED: recorded owner/child absent; no automatic retry or budget refund',
                        'time':time.time()},out)
            pending.append({'id':run_id,'kind':reserved['kind'],'status':status})
        return {'usage':budget.usage(out),'pending':pending,'reconciled':reconcile,
                'policy':'Completed artifacts require identity/hash validation; interrupted runs never resume estimator state implicitly.'}

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--reconcile',action='store_true');a=p.parse_args()
    print(json.dumps(resume(a.reconcile),indent=2))
