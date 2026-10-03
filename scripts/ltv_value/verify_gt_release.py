"""Verify local GT against currently linked official ZIP, fetching only ZIP index and GT."""
import io,re,zipfile,hashlib,requests
from common import *
class RemoteZipFile(io.RawIOBase):
 def __init__(self,url):
  self.session=requests.Session();self.session.trust_env=False;self.url=url;self.pos=0;self.bytes=0
  r=self.session.get(url,headers={'Range':'bytes=0-0'},stream=True,timeout=20)
  if r.status_code!=206:r.close();raise RuntimeError('server must honor ranges')
  self.url=r.url;self.size=int(r.headers['Content-Range'].split('/')[-1]);r.close()
 def seekable(self):return True
 def readable(self):return True
 def tell(self):return self.pos
 def seek(self,offset,whence=0):
  self.pos=offset if whence==0 else self.pos+offset if whence==1 else self.size+offset;return self.pos
 def read(self,n=-1):
  n=min(n if n>=0 else self.size-self.pos,self.size-self.pos)
  if n>16*1024*1024:raise RuntimeError('unexpected large range')
  if n==0:return b''
  r=self.session.get(self.url,headers={'Range':f'bytes={self.pos}-{self.pos+n-1}'},stream=True,timeout=20)
  if r.status_code!=206:r.close();raise RuntimeError('server ignored range')
  data=r.content;r.close();assert len(data)==n;self.pos+=n;self.bytes+=n;return data
if __name__=='__main__':
 html=(OUT/'provenance/datasets.html').read_text();result={}
 for seq in DEV+VALIDATION:
  urls=[u for u in re.findall(r'href=["\']([^"\']+)',html) if u.endswith(seq+'_snapdragon_with_gt.zip')];assert len(urls)==1
  source=RemoteZipFile(urls[0]);archive=zipfile.ZipFile(source);name=[n for n in archive.namelist() if n.endswith('/groundtruth.txt') or n=='groundtruth.txt'];assert len(name)==1
  content=archive.read(name[0]);remote=hashlib.sha256(content).hexdigest();local=sha(read(OUT/'inputs.json')[seq]['gt'])
  result[seq]={'url':urls[0],'resolved_url':source.url,'remote_sha':remote,'local_sha':local,'match':remote==local,'bytes_fetched':source.bytes,'zip_member':name[0],'zip_timestamp':archive.getinfo(name[0]).date_time}
  print(seq,result[seq],flush=True);write(OUT/'provenance/gt_release_verification.json',result)
  if remote!=local:raise RuntimeError('local GT does not match current official release; keep original and review limitations')
