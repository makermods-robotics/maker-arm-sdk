import math
from tools.diagnostics.encoder_gui import RangeStats, _fmt


def test_range_stats_tracks_high_resolution_motion():
    stats = RangeStats()
    for value in (1.0, 1.0004, 0.998, 1.025):
        stats.update(value)
    assert stats.samples == 4
    assert stats.start == 1.0
    assert stats.previous == 1.025
    assert stats.minimum == 0.998
    assert stats.maximum == 1.025
    assert math.isclose(stats.travel, 0.027)
    assert math.isclose(stats.delta, 0.025)
    assert math.isclose(stats.peak_step, 0.027)


def test_range_stats_ignores_nonfinite_samples():
    stats = RangeStats()
    stats.update(math.nan)
    stats.update(math.inf)
    assert stats.samples == 0
    assert math.isnan(stats.travel)
    assert _fmt(math.nan) == "--"
    assert _fmt(0.0004) == "+0.000400"
