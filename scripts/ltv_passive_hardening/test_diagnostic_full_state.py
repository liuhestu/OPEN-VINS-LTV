"""Same actual Adapter with heavy/light logging, exact x/P/slot comparison."""
import argparse
import json
from pathlib import Path
import tempfile
import numpy as np
import synthetic_inputs
from synthetic_runner import Adapter


def check(library):
    with tempfile.TemporaryDirectory() as folder:
        p=Path(folder)/'inputs'
        synthetic_inputs.generate(p,synthetic_inputs.ROOT/'config/ltv_euroc/kalibr_imucam_chain.yaml',
                                  'REGULAR',42,.5,'STEREO',3.)
        (p/'labels.npz').rename(p/'labels_estimator_unavailable.npz')
        with np.load(p/'inputs.npz') as data:inputs={key:data[key] for key in data.files}
        heavy=Adapter(library,3,'HYBRID',float(inputs['bearing_sigma_rad']),True)
        light=Adapter(library,3,'HYBRID',float(inputs['bearing_sigma_rad']),False)
        cursor=0;births=retirements=0;max_corrections=0;frames=0
        try:
            for k,t in enumerate(inputs['camera_times']):
                end=int(np.searchsorted(inputs['imu_sample_times'],t,side='right'))
                end=min(end+1,len(inputs['imu_sample_times']))
                while cursor<end:
                    for adapter in (heavy,light):adapter.imu(inputs['imu_sample_times'][cursor],inputs['imu_samples'][cursor])
                    cursor+=1
                a=heavy.frame(inputs,k);b=light.frame(inputs,k)
                for key in ('epoch','sequence','cursor','raw_current','ready_G','ready_V','raw_v','raw_eta','births','retirement_events','resource_diagnostics','camera_substeps','corrected_ids'):
                    assert a[key]==b[key],key
                assert len(a['corrected_ids'])==len(set(a['corrected_ids']))
                if not a['raw_current'] or a['camera_substeps']==0:
                    assert not a['corrected_ids']
                xa,Pa,ia,da=heavy.state(True);xb,Pb,ib,db=light.state(True)
                assert da==db
                np.testing.assert_array_equal(xa,xb)
                np.testing.assert_array_equal(Pa,Pb)
                np.testing.assert_array_equal(ia,ib)
                assert not b['heavy_diagnostics'] and not b['seeds'] and not b['tracks']
                births+=len(a['births']);retirements+=len(a['retirement_events'])
                max_corrections=max(max_corrections,a['actual_corrections']);frames+=1
        finally:
            heavy.close();light.close()
        assert frames==61 and births>0 and retirements>0 and max_corrections>0
        return {'status':'PASS','duration':3.,'frames':frames,'births':births,'retirements':retirements,
                'max_actual_corrections':max_corrections,'comparison':'Exact complete x/P/physical slot IDs every camera; same inputs; only logging differs'}


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--library',required=True)
    print(json.dumps(check(p.parse_args().library),indent=2))
