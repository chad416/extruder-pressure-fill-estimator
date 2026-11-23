"""FMEA row F3: slow zero drift of the head-pressure transducer (results/drift.md)."""

import numpy as np

from xtrude.control import PIDRuntime
from xtrude.ident import TS
from xtrude.plant import nominal
from xtrude.sim import Event, Scenario, run
from xtrude.zero_check import RunoutZeroCheck


def pid() -> PIDRuntime:
    return PIDRuntime(0.108, 0.472, 0.169, 2, 45, 0.5)


def test_zero_offset_shifts_the_reading_not_the_pressure():
    a, b = nominal(seed=1), nominal(seed=1)
    b.p_zero_bar = 5.0
    ra = np.array([a.measure()["P_bar"] for _ in range(200)])
    rb = np.array([b.measure()["P_bar"] for _ in range(200)])
    assert a.P == b.P
    assert abs((rb - ra).mean() - 5.0) < 0.05


def _pair(offset: float):
    ev = [Event(60, "eps", 0.94), Event(200, "eps", 1.0)]
    base = run(Scenario(400.0, list(ev)), pid(), use_ekf=True, plant_seed=7)
    drift = run(Scenario(400.0, [Event(0.0, "pz", offset)] + ev), pid(), use_ekf=True, plant_seed=7)
    return base, drift


def test_loop_holds_the_reading_so_true_pressure_and_throughput_move():
    base, drift = _pair(5.0)
    s = base.arr("t") > 150
    assert abs(drift.arr("P_meas")[s].mean() - 60.0) < 0.3
    assert abs(drift.arr("P")[s].mean() - 55.0) < 0.5
    dq = drift.arr("Q_out")[s].mean() / base.arr("Q_out")[s].mean() - 1.0
    assert -0.012 < dq < -0.004  # 0.12-0.18 % per bar on the assumed fill curve


def test_pressure_innovation_does_not_reveal_a_zero_offset():
    """The estimator explains the shifted reading with its free states, so the innovation
    stays where it was. This test documents that blindness: it fails if a later estimator
    makes the offset visible, and then the FMEA detection column must be rewritten."""
    base, drift = _pair(8.0)
    s = base.arr("t") > 150
    assert abs(drift.arr("nu_p")[s].mean() - base.arr("nu_p")[s].mean()) < 0.1
    # ... while the estimate itself is biased by the offset
    shift = np.mean((drift.arr("M_hat") - drift.arr("M"))[s]) - np.mean((base.arr("M_hat") - base.arr("M"))[s])
    assert 0.03 < shift < 0.3


def _runout(offset: float):
    e = nominal(seed=4)
    e.p_zero_bar = offset
    ctl = pid()
    ctl.init(60.0, e.measure()["P_bar"], 30)
    chk = RunoutZeroCheck()
    captured_in_production = False
    p_true_at_capture = None
    for k in range(int(420 / TS)):
        if k == int(200 / TS):
            e.feed_on = False
        m = e.measure()
        got = chk.step(e.feed_on, m["torque"], m["P_bar"], TS)
        if got is not None:
            p_true_at_capture = e.P / 1e5
            captured_in_production = e.feed_on
        e.step(ctl.step(60.0, m["P_bar"]), TS)
    return chk, captured_in_production, p_true_at_capture


def test_runout_zero_check_recovers_the_offset():
    for offset in (0.0, 4.0):
        chk, in_production, p_true = _runout(offset)
        assert chk.zero_bar is not None and not in_production
        assert p_true < 0.5  # the head had drained when the value was latched
        assert abs(chk.zero_bar - offset) < 0.3
        assert chk.alarm == (offset > 2.0)


def test_zero_check_stays_quiet_while_the_screw_is_loaded():
    e = nominal(seed=5)
    chk = RunoutZeroCheck()
    for _ in range(int(60 / TS)):
        m = e.measure()
        # feed flag lost while the screw is full: the torque condition must hold the check off
        assert chk.step(False, m["torque"], m["P_bar"], TS) is None
        e.step(30.0, TS)
    assert chk.zero_bar is None
