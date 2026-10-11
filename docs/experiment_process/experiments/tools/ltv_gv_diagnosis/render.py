"""Export unit-separated tables and full-time plots from offline derived evidence."""
import argparse
import csv
import json
from pathlib import Path
import shutil
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def main():
    p = argparse.ArgumentParser()
    p.add_argument('out', type=Path)
    p.add_argument('report', type=Path)
    args = p.parse_args()
    args.report.mkdir(parents=True, exist_ok=True)
    figures = args.report/'figures'
    figures.mkdir(exist_ok=True)
    summary = json.loads((args.out/'summary.json').read_text())
    table = []
    for seq, s in summary.items():
        for mode, m in s['modes'].items():
            for branch, groups in m.items():
                for group, z in groups.items():
                    r = dict(sequence=seq, mode=mode, branch=branch, stratum=group,
                             target_frames=z['post_comparison']['target_frames'], ready=z['ready_frames'],
                             attempts=z['attempts'], actual=z['actual_frames'])
                    for phase in ('post_comparison', 'prior_comparison', 'actual_prior_comparison'):
                        c = z[phase]
                        r[phase+'_paired'] = c['paired_frames']
                        r[phase+'_missing'] = c['missing_pairs']
                        r[phase+'_better_fraction'] = c['ltv_better_fraction']
                        r[phase+'_correlation'] = c['correlation']
                        for estimator in ('main', 'ltv'):
                            for statistic in ('rms', 'p50', 'p95'):
                                r[phase+'_'+estimator+'_'+statistic] = c[estimator][statistic]
                    for metric in ('residual_attempted', 'residual_actual', 'nis_attempted', 'nis_actual'):
                        for statistic in ('p50', 'p95', 'maximum'):
                            r[metric+'_'+statistic] = z[metric][statistic]
                    for quantity in ('gains', 'corrections'):
                        for block, values in z[quantity].items():
                            for statistic in ('p50', 'p95', 'maximum'):
                                r[quantity+'_'+block+'_'+statistic] = values[statistic]
                    table.append(r)
        raw = np.genfromtxt(args.out/(seq+'_curves.csv'), delimiter=',', names=True, dtype=None, encoding='utf-8')
        for b in ('G', 'V'):
            off = raw[(raw['mode']=='OFF') & (raw['branch']==b)]
            gv = raw[(raw['mode']=='GV') & (raw['branch']==b)]
            unit = 'deg' if b=='G' else 'm/s'
            fig, axes = plt.subplots(3, 1, figsize=(13, 8), sharex=True)
            axes[0].plot(off['elapsed_s'], off['main_error'], label='OFF main post', lw=.8)
            axes[0].plot(off['elapsed_s'], off['ltv_error'], label='OFF LTV', lw=.8)
            axes[0].plot(gv['elapsed_s'], gv['prior_error'], label='GV main prior', lw=.6, alpha=.5)
            axes[0].set_yscale('log')
            axes[0].set_ylabel('Error ('+unit+', log)')
            axes[0].legend(ncol=3)
            axes[1].plot(off['elapsed_s'], off['visual_rows'], label='OFF visual rows', lw=.8)
            axes[1].plot(off['elapsed_s'], off['observed_features'], label='LTV observed', lw=.8)
            axes[1].axhline(s['weak_visual_threshold'], color='gray', ls='--', label='Weak visual cutoff')
            axes[1].axhline(15, color='purple', ls=':', label='Existing readiness minimum')
            axes[1].set_ylabel('Rows / features')
            axes[1].legend(ncol=4)
            for key, y in [('ready', 2), ('actual', 1), ('rising_high_error', 0)]:
                selected = gv[key].astype(bool)
                axes[2].scatter(gv['elapsed_s'][selected], np.full(selected.sum(), y), s=3, label=key)
            axes[2].set_yticks([0, 1, 2], ['High & rising', 'Applied', 'Ready'])
            axes[2].set_ylim(-.5, 2.5)
            axes[2].set_xlabel('Seconds since first frozen camera packet')
            fig.suptitle(seq+' '+b+' physical errors and availability (all common support)')
            fig.tight_layout()
            fig.savefig(figures/(seq+'_'+b+'_errors.png'), dpi=130)
            plt.close(fig)
            fig, axes = plt.subplots(5, 1, figsize=(13, 10), sharex=True)
            axes[0].plot(gv['elapsed_s'], gv['residual_norm'], '.', ms=2)
            axes[0].set_ylabel('Residual\n'+('dimensionless' if b=='G' else 'm/s'))
            axes[1].plot(gv['elapsed_s'], gv['nis'], '.', ms=2)
            axes[1].set_ylabel('NIS')
            for ax, block, label in zip(axes[2:], ['attitude_deg', 'velocity_mps', 'position_m'], ['Attitude (deg)', 'Velocity (m/s)', 'Position (m)']):
                ax.plot(gv['elapsed_s'], gv['correction_'+block], '.', ms=2)
                ax.set_ylabel(label)
            axes[-1].set_xlabel('Seconds since first frozen camera packet')
            fig.suptitle(seq+' GV '+b+' actual branch contributions; missing values = no branch update')
            fig.tight_layout()
            fig.savefig(figures/(seq+'_'+b+'_updates.png'), dpi=130)
            plt.close(fig)
    with (args.report/'statistics.csv').open('w') as stream:
        w = csv.DictWriter(stream, fieldnames=list(table[0]), lineterminator='\n')
        w.writeheader()
        w.writerows(table)
    for name in ('summary.json', 'supply.json', 'read_manifest.json', 'freeze.json'):
        shutil.copyfile(args.out/name, args.report/name)


if __name__ == '__main__':
    main()
