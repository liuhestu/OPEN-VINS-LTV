"""Mechanism checks: current/future target mutations cannot change current gate."""
import copy
from analyze_gates import lagged_quality


def row(t, camera, feature=1, epoch=1):
    return dict(camera_ns=str(t), camera_id=str(camera), feature_id=str(feature),
                epoch=str(epoch), core_id=str(feature), entered='0.0',
                ltv_valid='1', past_fit_valid='1', ltv_angle_rad='0.01', past_fit_angle_rad='0.02')


def score(rs, consumed=None):
    times = sorted({int(r['camera_ns']) for r in rs})
    mapping = {t: t for t in times}
    events = {t: dict(epoch=1, corrected_ids=set([1]) if consumed is None else set(consumed.get(t, []))) for t in times}
    return lagged_quality(rs, mapping, events)[0].tolist()


def main():
    rows = [row(t, c) for t in [1, 2, 3] for c in [0, 1]]
    assert score(rows) == [False, False, True, True, True, True]
    # Change both models' current target scores and every future target score.
    # The current mask (both cameras) remains fixed; only the next-frame mask changes.
    modified = copy.deepcopy(rows)
    for r in modified[2:]:
        r.update(ltv_angle_rad='0.1', past_fit_angle_rad='0.001')
    assert score(modified)[:4] == score(rows)[:4]
    assert score(modified)[4:] == [False, False]
    # Camera row order cannot let current camera0 influence current camera1.
    shuffled = [rows[i] for i in [1, 0, 3, 2, 5, 4]]
    assert score(shuffled) == score(rows)
    # No actual consumption receipt means no score can be committed.
    assert score(rows, {1: [], 2: [1], 3: [1]})[2:4] == [False, False]
    # Birth/epoch identity transitions never reuse an old point's score.
    new_identity = copy.deepcopy(rows)
    for r in new_identity[2:4]:
        r['entered'] = '2.0'
    assert score(new_identity)[2:4] == [False, False]
    # An absent preceding packet cannot be bypassed by an older favourable score.
    missing = [row(1, 0), row(1, 1), row(2, 0, feature=2), row(2, 1, feature=2), row(3, 0), row(3, 1)]
    assert score(missing)[4:] == [False, False]
    print('PASS lagged prefix causality / camera order / consumption / identity / missing packet')


if __name__ == '__main__':
    main()
