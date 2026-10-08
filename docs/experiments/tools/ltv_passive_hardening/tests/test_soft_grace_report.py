import math
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from soft_grace_report import summarize


def row(k,t,held=True,count=1,last=0.):
    return dict(receipt_id=k,epoch=1,t=t,imu_time=t,camera_ns=round(t*1e9),raw_current=True,
                ready_G=True,ready_V=True,ready_soft_grace_enabled=True,grace_G=held,grace_V=held,
                soft_failure_frames_G=count,soft_failure_frames_V=count,last_strict_good_G=last,last_strict_good_V=last)

class GraceReportTest(unittest.TestCase):
    def test_two_held_unreferenced_and_ready_denominator(self):
        r=summarize([row(1,0,False,0),row(2,.05),row(3,.1,True,2)])['branches']['V']
        self.assertEqual(r['constraint_status'],'PASS');self.assertEqual(r['held_fraction_of_ready'],2/3)
        self.assertEqual(r['longest_consecutive_held_packets'],2);self.assertIsNone(r['held_severe_fraction'])
    def test_third_and_refresh_fail(self):
        r=summarize([row(1,.02),row(2,.04,True,2),row(3,.06,True,3,.04)])['branches']['G']
        self.assertEqual(r['constraint_status'],'FAIL');self.assertEqual(r['held_packets'],3)
    def test_large_epoch_one_ulp(self):
        base=1.4e9;end=math.nextafter(base+.1,math.inf)
        self.assertEqual(summarize([row(1,end,last=base)])['branches']['G']['constraint_status'],'PASS')
        self.assertEqual(summarize([row(1,math.nextafter(end,math.inf),last=base)])['branches']['G']['constraint_status'],'FAIL')
    def test_existing_synthetic_errors_no_support_change(self):
        rows=[row(1,.05),row(2,.1,True,2)]
        curves=dict(t=np.array([.05,.1]),ready_G=np.array([1,1]),ready_V=np.array([1,1]),angle=np.array([6.,np.nan]),v=np.array([.5,2.]))
        r=summarize(rows,curves)
        self.assertEqual(r['branches']['G']['reference_missing_packets'],1)
        self.assertEqual(r['branches']['G']['held_severe_fraction'],1)
        self.assertEqual(r['branches']['V']['held_severe_fraction'],.5)
    def test_native_join_and_wrong_ready_rejected(self):
        rows=[row(1,.05)]
        curves=dict(camera_ns=np.array([50000000]),time=np.array([.05]),P_NEW_ready_G=np.array([1]),P_NEW_ready_V=np.array([1]),
                    P_NEW_error_g_angle_deg=np.array([.5]),P_NEW_error_v=np.array([.2]))
        self.assertEqual(summarize(rows,curves)['branches']['V']['reference_available_packets'],1)
        curves['P_NEW_ready_V'][0]=0
        with self.assertRaisesRegex(ValueError,'support mismatch'):summarize(rows,curves)
    def test_no_nearest_camera_or_time_matching(self):
        rows=[row(1,.05)]
        curves=dict(camera_ns=np.array([50000001]),time=np.array([.05]),P_NEW_ready_G=np.array([1]),P_NEW_ready_V=np.array([1]),
                    P_NEW_error_g_angle_deg=np.array([.5]),P_NEW_error_v=np.array([.2]))
        self.assertEqual(summarize(rows,curves)['branches']['G']['reference_available_packets'],0)
        curves['camera_ns'][0]=50000000;curves['time'][0]=.051
        with self.assertRaisesRegex(ValueError,'physical time mismatch'):summarize(rows,curves)
        synthetic=dict(t=np.array([math.nextafter(.05,math.inf)]),ready_G=np.array([1]),ready_V=np.array([1]),angle=np.array([.5]),v=np.array([.2]))
        self.assertEqual(summarize(rows,synthetic)['branches']['V']['reference_available_packets'],0)
    def test_old_missing_log_is_unavailable(self):
        a=row(1,0);del a['grace_G']
        r=summarize([a])['branches']['G']
        self.assertEqual(r['constraint_status'],'UNAVAILABLE_INCOMPLETE_LOG');self.assertIsNone(r['held_error_RMSE'])

if __name__=='__main__':unittest.main()
