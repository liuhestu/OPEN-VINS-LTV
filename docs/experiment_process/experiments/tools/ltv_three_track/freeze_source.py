#!/usr/bin/env python3
"""Publish an immutable committed-source tree for one isolated build identity."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import subprocess
import tarfile

p=argparse.ArgumentParser();p.add_argument('coord',type=Path);p.add_argument('task');p.add_argument('oid');a=p.parse_args()
session=json.loads((a.coord/'session.json').read_text());task=session['tasks'][a.task];wt=Path(task['worktree']);root=Path(task['task_root'])
oid=subprocess.check_output(['git','rev-parse',a.oid],cwd=wt,text=True).strip();dest=root/'source'/oid
with (root/'task_artifact.lock').open('a') as lock:
 fcntl.flock(lock,fcntl.LOCK_EX)
 if dest.exists():
  if not (dest/'source_identity.json').exists():raise RuntimeError('partial existing snapshot')
  print(dest);raise SystemExit(0)
 archive=root/'tmp'/f'{oid}.tar'
 with archive.open('wb') as stream:subprocess.run(['git','archive','--format=tar',oid],cwd=wt,stdout=stream,check=True)
 dest.mkdir()
 with tarfile.open(archive) as stream:
  members=stream.getmembers()
  if any(m.name.startswith('/') or '..' in Path(m.name).parts or m.isdev() for m in members):raise RuntimeError('unsafe archive')
  stream.extractall(dest)
 (dest/'source_identity.json').write_text(json.dumps({'source_oid':oid,'worktree':str(wt),'immutable':True},indent=2)+'\n')
 for path in dest.rglob('*'):
  if path.is_file() and not path.is_symlink():path.chmod(path.stat().st_mode & ~0o222)
 archive.unlink()
 print(dest)
