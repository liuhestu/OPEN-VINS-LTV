#!/usr/bin/env python3
import json
from pathlib import Path
import tempfile
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from final_euroc import Final, PROFILES, MODES

class FinalMetrics(unittest.TestCase):
    def test_profiles(self):
        self.assertEqual(PROFILES['MH']['G'],(16,0,0))
        self.assertEqual(PROFILES['MH']['GVL'],(2,49,6))
        self.assertEqual(PROFILES['Vicon']['GVL'],(1,1,1))
        self.assertEqual(PROFILES['Vicon']['V'],(0,6,0))
    def test_rigid_gauge_and_rpe(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);(root/'support').mkdir();times=np.arange(0,6.01,.05);truth=np.column_stack([times,np.sin(times),np.cos(times)])
            np.savez(root/'support/MH_01_easy.npz',times=times,truth=truth,initialization=0.)
            gt=np.zeros((len(times),17));gt[:,0]=times*1e9;gt[:,1:4]=truth;gt[:,4]=1;np.savetxt(root/'gt.csv',gt,delimiter=',')
            obj=object.__new__(Final);obj.coord=root;obj.inputs={'MH_01_easy':{'gt':str(root/'gt.csv')}};obj.index=[]
            for mode in MODES:
                d=root/mode;d.mkdir();x=np.zeros((len(times),17));x[:,0]=times;x[:,4]=1;x[:,5:8]=truth
                if mode=='G':x[:,5:8]+=np.array([1,2,3])
                if mode=='V':
                    rotation=Rotation.from_rotvec([.2,.1,.3]);x[:,1:5]=rotation.as_quat();x[:,5:8]=rotation.apply(truth)+np.array([2,-3,1])
                np.savetxt(d/'trajectory.csv',x,delimiter=',',header='columns',comments='');obj.index.append(dict(sequence='MH_01_easy',mode=mode,status='VALID_FULL_MATCHED',output_dir=str(d),ate_rmse_m=0.))
            # The evaluator iterates the frozen real sequence list; restrict this test locally.
            import final_euroc
            original=final_euroc.EUROC
            try:
                final_euroc.EUROC=['MH_01_easy'];obj.evaluate_metrics()
            finally:final_euroc.EUROC=original
            for row in json.loads((root/'metrics.json').read_text()):
                self.assertLess(row['position_max_m'],1e-10)
                for dt in [1,5]:
                    self.assertLess(row[f'rpe_translation_{dt}s_m'],1e-10)
                    self.assertLess(row[f'rpe_rotation_{dt}s_deg'],1e-10)
                    self.assertGreater(row[f'rpe_samples_{dt}s'],0)

if __name__=='__main__':unittest.main()
