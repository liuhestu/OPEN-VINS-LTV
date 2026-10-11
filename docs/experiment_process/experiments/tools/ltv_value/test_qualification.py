import unittest,copy
from unittest.mock import patch
from analyze import qualification

def fixture():
 good={'seconds':10,'improvement':.1,'ci95':[.02,.2],'baseline_p95':1.,'method_p95':.9}
 bad={'seconds':10,'improvement':-.1,'ci95':[-.2,-.02],'baseline_p95':1.,'method_p95':1.05}
 result=[]
 for i in range(3):
  branch={'own':copy.deepcopy(good if i<2 else bad),'incremental':copy.deepcopy(bad),'sensitivity':{str(w):{'own':copy.deepcopy(good),'incremental':copy.deepcopy(bad)} for w in [.05,.1,.2]}}
  result.append({'sequence':str(i),'strata':{'all':{'G':copy.deepcopy(branch),'V':copy.deepcopy(branch)}}})
 return result
class GateTests(unittest.TestCase):
 @patch('analyze.write')
 def test_two_of_three(self,_):
  q=qualification(fixture());self.assertTrue(q['G']['qualified']);self.assertTrue(q['V']['qualified'])
 @patch('analyze.write')
 def test_reference_sensitive(self,_):
  r=fixture();r[1]['strata']['all']['V']['sensitivity']['0.2']['own']['improvement']=-.01
  q=qualification(r);self.assertFalse(q['V']['qualified']);self.assertTrue(q['G']['qualified'])
 @patch('analyze.write')
 def test_short_or_harm(self,_):
  r=fixture();r[0]['strata']['all']['G']['own']['seconds']=4
  r[0]['strata']['all']['V']['incremental']['method_p95']=1.2
  q=qualification(r);self.assertFalse(q['G']['qualified']);self.assertFalse(q['V']['qualified'])
if __name__=='__main__':unittest.main()
