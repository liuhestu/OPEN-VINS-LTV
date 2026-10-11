#!/usr/bin/env python3
"""Terminate only a registry-owned process group with fresh PID/start verification."""
import argparse
import json
import os
from pathlib import Path
import signal
import time
from launcher import group

p=argparse.ArgumentParser();p.add_argument('coord',type=Path);p.add_argument('task');p.add_argument('run_id');p.add_argument('--reason',required=True);a=p.parse_args()
registry=a.coord/'registry'/f'task_{a.task}.jsonl';records=[json.loads(s) for s in registry.read_text().splitlines()]
owned=[r for r in records if r.get('run_id')==a.run_id];starts=[r for r in owned if r.get('event')=='START']
if len(starts)!=1:raise RuntimeError('one owned START required')
start=starts[0]
if str(os.getuid())!=start['owner']:raise RuntimeError('wrong owner')
pgid=start['pgid'];expected={(r.get('pid'),r.get('start_ticks')) for r in owned if r.get('event') in ('PROCESS_REGISTERED','EXEC_READY')}
if start.get('start_identity'):expected.add((start['pid'],start['start_identity']['start_ticks']))
live=group(pgid)
if not live:print('already terminal');raise SystemExit(0)
if not any((r['pid'],r['start_ticks']) in expected for r in live):raise RuntimeError('no live matching registered identity; refuse signal')
with registry.open('a') as f:f.write(json.dumps({'event':'STOP_REQUEST','run_id':a.run_id,'task':a.task,'pgid':pgid,'reason':a.reason,'verified_live':live,'time':time.time()})+'\n')
os.killpg(pgid,signal.SIGTERM)
print('SIGTERM sent to verified owned group; launcher retains locks until terminal')
