import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from point_lifetimes import summarize


def row(receipt,time,epoch=1):
    return dict(receipt_id=receipt,imu_time=time,epoch=epoch,current_cam0_ids=[9],available=True,raw_current=True,
                camera_substeps=3,corrected_ids=[],tracks=[],seeds=[],births=[])


class LifetimeTest(unittest.TestCase):
    def test_exact_corrections_not_visibility_or_substeps(self):
        a=row(1,0);b=row(2,.1);c=row(3,.2);d=row(4,.3)
        b['births']=[dict(id=9,apply_seed=True,source='STEREO')]
        b['seeds']=[dict(id=9,admitted=True,mean_written=True,time=.1,source='STEREO')]
        b['tracks']=[dict(id=9,first_seen=0,entered=.1,seeded=.1,last_seen=.1,ever_opportunity=True,phase='ACTIVE')]
        b['corrected_ids']=[9];c['corrected_ids']=[9];d['current_cam0_ids']=[]
        d['retirement_events']=[dict(id=9,epoch=1,kind='ACTIVE_RETIRED',time=.3)]
        p=summarize([a,b,c,d])['points'][0]
        self.assertEqual(p['actual_correction_events'],2);self.assertEqual(p['seed_events_observed'],1)
        self.assertAlmostEqual(p['observation_age_at_last_observation'],.2)
        self.assertAlmostEqual(p['residence_age_at_exit_or_censor'],.2)
        self.assertFalse(p['right_censored']);self.assertIsNone(p['error_metrics'])
    def test_missing_old_field_is_unavailable(self):
        a=row(1,0);del a['corrected_ids']
        p=summarize([a])['points'][0]
        self.assertIsNone(p['actual_correction_events']);self.assertIn('UNAVAILABLE',p['correction_count_status'])
        self.assertTrue(p['right_censored']);self.assertIsNone(p['admission'])
    def test_repeat_seed_refused_new_epoch_allowed(self):
        a=row(1,0);a['births']=[dict(id=9,apply_seed=True)]
        b=row(2,.1);b['births']=[dict(id=9,apply_seed=True)]
        with self.assertRaises(ValueError):summarize([a,b])
        b['epoch']=2
        self.assertEqual(summarize([a,b])['point_count'],2)
    def test_ttl_is_not_admitted_retirement(self):
        a=row(1,0);b=row(2,2.1);b['current_cam0_ids']=[]
        b['retirement_events']=[dict(id=9,epoch=1,kind='CANDIDATE_TTL',time=2.1)]
        p=summarize([a,b])['points'][0]
        self.assertIsNone(p['residence_age_at_exit_or_censor']);self.assertEqual(p['retirement_kind'],'CANDIDATE_TTL')
    def test_invalid_correction_evidence_refused(self):
        a=row(1,0);a['corrected_ids']=[9];a['camera_substeps']=0
        with self.assertRaisesRegex(ValueError,'actual available'):summarize([a])
    def test_birth_without_seed_is_still_admission(self):
        a=row(1,0);a['births']=[dict(id=9,apply_seed=False)]
        p=summarize([a])['points'][0]
        self.assertEqual(p['admission'],0);self.assertIsNone(p['seed_time']);self.assertEqual(p['seed_events_observed'],0)

if __name__=='__main__':unittest.main()
