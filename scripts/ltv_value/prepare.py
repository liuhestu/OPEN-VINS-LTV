"""Read-only source audit and lossless sensor-only ASL mirror. No GT in replay roots."""
import argparse, decimal, json, re, shutil
from pathlib import Path
import numpy as np
import cv2, yaml
from common import *
def ns(text):
    value=decimal.Decimal(text)*decimal.Decimal(1000000000)
    return int(value.to_integral_value(rounding=decimal.ROUND_HALF_EVEN))
def lines(path):
    return [line.split() for line in path.read_text().splitlines() if line.strip() and not line.startswith('#')]
def load_yaml(p): return yaml.safe_load('\n'.join(x for x in p.read_text().splitlines() if not x.startswith('%')))
def monotonic(t):
    if len(t)<2 or any(b<=a for a,b in zip(t,t[1:])): raise ValueError('nonmonotonic input')
def prepare():
    if (OUT/'inputs.json').exists(): raise SystemExit('already frozen; use --check')
    inputs={}; calibration={}
    for seq in SEQUENCES:
        euroc=seq in EUROC
        root=Path('/home/he/datasets/euroc/ASL')/seq/'mav0' if euroc else Path('/home/he/datasets/uzhfpv/archives')/(seq+'_snapdragon_with_gt')
        cfg='ltv_euroc' if euroc else 'uzhfpv_indoor_45' if seq.startswith('indoor_45') else 'uzhfpv_indoor' if seq.startswith('indoor') else 'uzhfpv_outdoor'
        dest=OUT/'inputs'/seq; dest.mkdir(parents=True,exist_ok=True)
        hashes={}; cams=[]; raw_times=[]; rounding_errors=[]
        for cam in [0,1]:
            src=root/f'cam{cam}/data.csv' if euroc else root/('left_images.txt' if cam==0 else 'right_images.txt')
            hashes[str(src)]=sha(src)
            rows=[(int(x[0]),root/f'cam{cam}/data'/x[1]) for x in (l.split(',') for l in src.read_text().splitlines() if l and not l.startswith('#'))] if euroc else [(ns(x[1]),root/x[2]) for x in lines(src)]
            original_times=[decimal.Decimal(str(t))/decimal.Decimal(1000000000) for t,_ in rows] if euroc else [decimal.Decimal(x[1]) for x in lines(src)]
            raw_times.append(dict(zip([t for t,_ in rows],original_times)))
            rounding_errors.extend(float(abs(u*decimal.Decimal(1000000000)-t)) for (t,_),u in zip(rows,original_times))
            monotonic([t for t,_ in rows]); cams.append(rows)
            target=dest/f'cam{cam}';(target/'data').mkdir(parents=True,exist_ok=True)
            csv=['#timestamp [ns],filename']
            for t,p in rows:
                img=cv2.imread(str(p),cv2.IMREAD_GRAYSCALE)
                if img is None: raise ValueError('missing/bad image '+str(p))
                hashes[str(p)]=sha(p)
                link=target/'data'/p.name
                if not link.exists(): link.symlink_to(p)
                csv.append(f'{t},{p.name}')
            (target/'data.csv').write_text('\n'.join(csv)+'\n')
        imu_src=root/'imu0/data.csv' if euroc else root/'imu.txt';hashes[str(imu_src)]=sha(imu_src)
        rows=[(int(x[0]),x[1:]) for x in (l.split(',') for l in imu_src.read_text().splitlines() if l and not l.startswith('#'))] if euroc else [(ns(x[1]),x[2:]) for x in lines(imu_src)]
        monotonic([t for t,_ in rows]);assert all(len(v)==6 and np.isfinite(np.array(v,float)).all() for _,v in rows)
        (dest/'imu0').mkdir(exist_ok=True);(dest/'imu0/data.csv').write_text('#t,wx,wy,wz,ax,ay,az\n'+'\n'.join(str(t)+','+','.join(v) for t,v in rows)+'\n')
        gt=root/'state_groundtruth_estimate0/data.csv' if euroc else root/'groundtruth.txt';hashes[str(gt)]=sha(gt)
        g=np.loadtxt(gt,delimiter=',' if euroc else None,comments='#');assert np.isfinite(g).all() and np.all(np.diff(g[:,0])>0)
        cfgdir=ROOT/'config/ltv_value'/cfg;cfgdir.mkdir(parents=True,exist_ok=True)
        original=ROOT/'config'/cfg
        text=(original/'estimator_config.yaml').read_text()
        for key,v in {'calib_cam_extrinsics':'false','calib_cam_intrinsics':'false','calib_cam_timeoffset':'false','max_slam':'0','try_zupt':'false','use_aruco':'false'}.items():
            text=re.sub(r'^'+key+r':.*$',key+': '+v,text,flags=re.M)
        if not euroc: text+='\n'+(ROOT/'config/ltv_euroc/estimator_config.yaml').read_text().split('# LTV experiment base:')[1].split('\n',1)[1]
        (cfgdir/'estimator_config.yaml').write_text(text)
        for p in original.iterdir():
            if p.name!='estimator_config.yaml' and p.suffix in ['.yaml','.png']: shutil.copyfile(p,cfgdir/p.name)
        native=load_yaml(original/'kalibr_imucam_chain.yaml'); offset=native['cam0'].get('timeshift_cam_imu',0.)
        if not euroc and cfg not in calibration:
            cal_name=seq.rsplit('_',1)[0]+'_calib_snapdragon'
            off=next((root.parent/cal_name).glob('camchain-imucam*.yaml'))
            official=load_yaml(off)
            diffs={}
            for cam in ['cam0','cam1']:
                for key in ['T_cam_imu','camera_model','distortion_model','distortion_coeffs','intrinsics','resolution','timeshift_cam_imu']:
                    if native[cam].get(key)!=official[cam].get(key):diffs[cam+'.'+key]={'repo':native[cam].get(key),'official':official[cam].get(key)}
            calibration[cfg]={'camera_differences':diffs,'official_camera':str(off),'official_camera_sha':sha(off),'native_noise':load_yaml(original/'kalibr_imu_chain.yaml'),'official_noise':load_yaml(root.parent/cal_name/'imu.yaml'),'decision':'retain repository estimator noise; camera calibration must match official','common_cam0_offset':offset,'cam1_offset':native['cam1'].get('timeshift_cam_imu')}
            if diffs: raise ValueError('camera calibration mismatch '+cfg)
        common=sorted(set(t for t,_ in cams[0]) & set(t for t,_ in cams[1]))
        if any(raw_times[0][t]!=raw_times[1][t] for t in common): raise ValueError('rounding would falsely pair asynchronous stereo')
        imu_ns=np.array([t for t,_ in rows],dtype=np.int64); targets=np.array(common)*1e-9+offset
        # Boundary failures are explicitly recorded, not hidden by trimming.
        bracket=(targets>=imu_ns[0]*1e-9)&(targets<imu_ns[-1]*1e-9)
        inputs[seq]={'dataset':'euroc' if euroc else 'uzh','split':'validation' if seq in VALIDATION else 'development','source':str(root),'replay_root':str(dest),'config':str(cfgdir),'gt':str(gt),'gt_rows':len(g),'gt_columns':g.shape[1],'duration':float((imu_ns[-1]-imu_ns[0])*1e-9),'imu_rows':len(rows),'paired_images':len(common),'left_images':len(cams[0]),'right_images':len(cams[1]),'max_image_rounding_ns':max(rounding_errors),'unbracketed_packets':int((~bracket).sum()),'offset':offset,'hashes':hashes,'derived_csv':{str(p):sha(p) for p in dest.rglob('*.csv')}}
        print(seq,'paired',len(common),'unbracketed',int((~bracket).sum()),flush=True)
    write(OUT/'inputs.json',inputs);write(OUT/'calibration_audit.json',calibration)
def check():
    for seq,x in read(OUT/'inputs.json').items():
        for p,h in {**x['hashes'],**x['derived_csv']}.items(): assert sha(p)==h,p
        print(seq,'unchanged',flush=True)
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--check',action='store_true');args=p.parse_args()
    check() if args.check else prepare()
