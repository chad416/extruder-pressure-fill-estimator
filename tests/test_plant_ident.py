import math

import numpy as np

from xtrude.ident import TS, fit_oe, step_test
from xtrude.physics import Pump, shape_factors
from xtrude.plant import RPM, nominal, operating_point


def test_shape_factors_limits():
    # shallow channel: both factors tend to 1 (Tadmor & Gogos Sec. 6.3)
    Fd, Fp = shape_factors(0.001, 1.0)
    assert abs(Fd - 1) < 0.002 and abs(Fp - 1) < 0.002
    Fd, Fp = shape_factors(0.018, 0.128)
    assert 0.9 < Fd < 0.95 and 0.88 < Fp < 0.94


def test_drag_flow_matches_hand_calculation():
    p = Pump()
    s = p.screw
    N = 0.5
    hand = 0.5 * math.pi * s.D * N * math.cos(s.phi) * s.W * s.H * p.Fd
    assert math.isclose(p.q_drag(N), hand, rel_tol=1e-12)


def test_steady_state_is_steady():
    for outlet in ("gearpump", "die"):
        e = nominal(outlet)
        t0 = e.truth()
        for _ in range(int(60 / TS)):
            e.step(30.0, TS)
        t1 = e.truth()
        assert abs(t1["P_bar"] - t0["P_bar"]) < 0.05
        assert abs(t1["M"] - t0["M"]) < 0.01
        assert abs(t1["T"] - t0["T"]) < 0.05


def test_design_point_plausible():
    e = nominal()
    tr = e.truth()
    q_kgh = tr["Q_s"] * e.p.compound.rho * 3600
    assert 600 < q_kgh < 900  # vendor class: 1100 kg/h at 45 rpm, no head pressure
    assert 0.5 < tr["torque"] / e.p.torque_rated < 0.85
    sec = tr["power"] / 1e3 / q_kgh
    assert 0.08 < sec < 0.3
    assert 100 < tr["T"] < 140


def test_feed_stop_mass_balance():
    """With the feed off, mass lost from the screw = volume pumped out (head ~ constant)."""
    e = nominal()
    e.feed_on = False
    m0 = e.truth()["M"]
    out = 0.0
    for _ in range(int(40 / TS)):
        e.step(30.0, TS)
        out += e.q_out(e.P, e.T) * TS * e.p.compound.rho
    m1 = e.truth()["M"]
    head = e.p.V_head / e.p.B_eff * (60e5 - e.P) * e.p.compound.rho  # compressibility term
    assert abs((m0 - m1) - (out - head)) / out < 0.05


def test_gearpump_speed_raises_pressure_and_feed_drop_is_delayed():
    e = nominal()
    for _ in range(int(3 / TS)):
        e.step(31.0, TS)
    assert e.truth()["P_bar"] > 62
    e = nominal()
    e.eps = 0.95
    tq0 = e.truth()["torque"]
    for _ in range(int(20 / TS)):
        e.step(30.0, TS)
    tr = e.truth()
    assert abs(tr["P_bar"] - 60) < 0.5  # pressure has not seen it yet
    assert tr["torque"] < tq0 * 0.99  # torque already has


def test_identification_small_amplitude_linear():
    rec = step_test(nominal(seed=1), amp_rpm=0.1, seed=1, duration=120.0, noise=False)
    m = fit_oe(rec)
    assert m.fit_pct > 95
    assert np.all(np.abs(m.poles) < 1)


def test_operating_point_sets_pressure():
    e = operating_point(P_bar=40.0, N_rpm=25.0)
    assert abs(e.truth()["P_bar"] - 40.0) < 0.5
    assert abs(e.N / RPM - 25.0) < 1e-9


def test_empty_screw_creates_no_material():
    """After a run-out empties the screw, the head must drain (0.1.0 held it at ~9 bar)."""
    from xtrude.control import PIDRuntime
    e = nominal(seed=2)
    pid = PIDRuntime(0.108, 0.472, 0.169, 2, 45, 0.5)
    pid.init(60, e.measure()["P_bar"], 30)
    e.feed_on = False
    for _ in range(int(150 / TS)):
        e.step(pid.step(60, e.measure()["P_bar"]), TS)
    assert e.P < 1e5 and e.mass() < 0.01
