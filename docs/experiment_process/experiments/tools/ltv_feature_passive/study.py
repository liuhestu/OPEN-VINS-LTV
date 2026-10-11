#!/usr/bin/env python3
"""Public phase entry points; all computation goes through the shared budget."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import budget

HERE=Path(__file__).resolve().parent

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('phase',choices=['prepare','calibrate','develop','freeze','validate','report','resume'])
    a,rest=p.parse_known_args()
    if a.phase=='prepare':
        if rest:p.error('prepare takes no extra arguments')
        from prepare import prepare
        print(json.dumps(prepare(),indent=2));return
    if a.phase=='calibrate':
        if rest:p.error('calibrate reuses the completed immutable diagnostic; no implicit rerun')
        from artifacts import sha
        path=budget.OUT/'geometry_calibration_01/calibration.json'
        finished=[e for e in budget.events() if e['event']=='finished' and e['id']==9]
        if not path.exists() or not finished or finished[-1]['exit_code']!=0:raise RuntimeError('Completed calibration unavailable')
        print(json.dumps({'status':'REUSED_DIAGNOSTIC','attempt':9,'path':str(path),'sha256':sha(path),
                          'claim':'Stratified development calibration only; no probability calibration claim'},indent=2));return
    module={'develop':'experiment.py','freeze':'freeze.py','validate':'validate.py','resume':'resume.py','report':'report_plots.py'}[a.phase]
    command=[sys.executable,str(HERE/module)]+rest
    if a.phase=='report':
        result=budget.run('analysis',command,'report_saved_evidence')
        print(json.dumps(result));raise SystemExit(result['exit_code'])
    raise SystemExit(subprocess.call(command))

if __name__=='__main__':main()
