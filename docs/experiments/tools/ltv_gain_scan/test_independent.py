#!/usr/bin/env python3
import unittest
from independent import weights, assess, REP

class IndependentContract(unittest.TestCase):
    def rows(self):
        return [dict(sequence=s,status='VALID_FULL_MATCHED',coverage=1,initialization_delta_s=0,ate_rmse_m=1.1,off_ate=1,reference_ate=1) for s in REP]
    def test_independent_axes(self):
        a,b=weights('GV',5,6),weights('GV',5,8)
        self.assertEqual(a['ltv_sigma_gravity_deg'],b['ltv_sigma_gravity_deg'])
        self.assertNotEqual(a['ltv_sigma_velocity_mps'],b['ltv_sigma_velocity_mps'])
        self.assertEqual(weights('V',0,6)['ltv_sigma_gravity_deg'],10)
    def test_high_singles_only(self):
        self.assertEqual(weights('G',16,0)['ltv_sigma_gravity_deg'],2.5)
        self.assertEqual(weights('V',0,16)['ltv_sigma_velocity_mps'],.25)
        with self.assertRaises(ValueError):weights('GV',10,16)
    def test_v25_only(self):
        self.assertEqual(weights('V',0,25)['ltv_sigma_velocity_mps'],.2)
        for args in [('G',25,0),('GV',2,25)]:
            with self.assertRaises(ValueError):weights(*args)
    def test_velocity_36_49_only(self):
        self.assertEqual(weights('V',0,36)['ltv_sigma_velocity_mps'],1/6)
        self.assertEqual(weights('V',0,49)['ltv_sigma_velocity_mps'],1/7)
        for args in [('G',36,0),('GV',2,49)]:
            with self.assertRaises(ValueError):weights(*args)
    def test_mode_identity(self):
        for args in [('V',2,6),('G',3,2),('GV',0,6),('GV',5,7)]:
            with self.assertRaises(ValueError):weights(*args)
    def test_strict_ten_percent(self):
        r=self.rows();self.assertEqual(assess(r,REP)['status'],'PASS')
        r[1]['ate_rmse_m']=1.1000000001;self.assertEqual(assess(r,REP)['status'],'BLOCKED')
    def test_invalid_support(self):
        for field,value in [('coverage',.999),('initialization_delta_s',None),('initialization_delta_s',float('nan')),('status','FAIL'),('reference_ate',0)]:
            r=self.rows();r[0][field]=value;self.assertEqual(assess(r,REP)['status'],'BLOCKED')
        self.assertEqual(assess(self.rows()[:-1],REP)['status'],'BLOCKED')

if __name__=='__main__':unittest.main()
