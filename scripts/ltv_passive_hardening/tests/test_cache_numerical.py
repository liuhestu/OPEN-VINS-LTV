"""No observer execution: decoder self parity and x tampering rejection."""
import json,subprocess,tempfile
from pathlib import Path
binary='/home/he/output/ltv_passive_hardening_v2/compare_cache_numerical'
cache='/home/he/output/ltv_passive_hardening_v2/R3_logger_cd3_short/HARDENED/cache.bin'
with tempfile.TemporaryDirectory() as d:
 a=str(Path(d)/'same.cache');b=str(Path(d)/'tamper.cache')
 subprocess.run([binary,'--fixture',cache,a,b],check=True)
 x=subprocess.run([binary,a,a],capture_output=True,text=True);assert x.returncode==0,x.stderr
 x=subprocess.run([binary,a,b],capture_output=True,text=True);assert x.returncode==1 and 'full x/P' in x.stderr,x.stderr
 x=subprocess.run([binary,cache,cache],capture_output=True,text=True);assert x.returncode==0,x.stderr
 report=json.loads(x.stdout);assert report['frames']==200
 print(json.dumps({'status':'PASS','self_comparison':report,'tampered_x_rejected':True,'observer_executed':False}))
