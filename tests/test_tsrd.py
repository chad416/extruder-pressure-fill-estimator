"""Second plant case: batch-fed twin-screw unit with a roller die (results/tsrd.md)."""

import numpy as np
import pytest

from xtrude.tsrd import RPM, Tsrd, TsrdParams
from xtrude.tsrd_sim import (MaterialModel, StartupConfig, batch_plan, learn_memory, run, startup_metrics,
                             u_roll_for)

P_SP = 20.0


def _gain(kappa: float) -> float:
    e = Tsrd(seed=0)
    e.steady(m_h=1e6, kappa=kappa)
    n0 = e.N / RPM
    for _ in range(600):
        e.step(n0, 0.1)
    p0 = e.P
    for _ in range(600):
        e.step(n0 + 0.3, 0.1)
    return (e.P - p0) / 1e5 / 0.3


def _startup(kappa: float, seed: int, mem=None, **kw):
    p = TsrdParams()
    cfg = StartupConfig(memory=mem, **kw)
    return run(batch_plan(seed, 3, p, kappa=kappa), cfg, duration=160, seed=seed, u_roll=u_roll_for(p, kappa, P_SP))


@pytest.fixture(scope="module")
def memory():
    return learn_memory(_startup(1.0, 910))


def test_mass_is_conserved_through_fill_pressure_and_run_out():
    e = Tsrd(seed=0)
    e.empty()
    e.drop_batch(220.0)
    worst = 0.0
    for k in range(4000):
        e.step(16.0 if k < 700 else 12.0, 0.1)
        worst = max(worst, abs(e.m_in - e.m_out - e.mass_total()))
    assert e.m_out > 100.0  # the unit did deliver
    assert worst < 1e-6


def test_no_pressure_until_the_chamber_is_full_then_a_jump():
    e = Tsrd(seed=0)
    e.empty()
    e.drop_batch(220.0)
    P = []
    for _ in range(1200):
        e.step(16.0, 0.1)
        P.append(e.P / 1e5)
    P = np.array(P)
    i = int(np.argmax(P > 0.5))
    assert i > 300  # more than 30 s of filling with no reading
    assert P[:i].max() < 0.5
    assert P[i + 50] > 10.0  # 5 s later the pressure is already above 10 bar


def test_plant_gain_grows_with_the_recipe_viscosity():
    g07, g10, g14 = _gain(0.7), _gain(1.0), _gain(1.4)
    assert g07 < g10 < g14
    assert g14 / g07 == pytest.approx(2.0, rel=0.1)


def test_roller_speed_matches_the_mixer_rate_at_the_setpoint():
    p = TsrdParams()
    for kappa in (0.7, 1.0, 1.4):
        e = Tsrd(p, seed=0)
        e.steady(kappa=kappa)
        e.u_roll = u_roll_for(p, kappa, P_SP)
        assert e.q_roll(P_SP * 1e5) == pytest.approx(p.Q_ref, rel=1e-6)


def test_plain_pi_start_up_settles(memory):
    m = startup_metrics(_startup(1.0, 3), P_SP)
    assert m["settled"]
    assert m["P_peak"] > 24.0  # the overshoot the recipe table is there to remove


def test_preload_and_revolution_count_cut_peak_and_settling(memory):
    base = startup_metrics(_startup(1.0, 3, memory), P_SP)
    pre = startup_metrics(_startup(1.0, 3, memory, use_preload=True), P_SP)
    both = startup_metrics(_startup(1.0, 3, memory, use_preload=True, use_rev_count=True), P_SP)
    assert base["P_peak"] > pre["P_peak"] > both["P_peak"]
    assert both["P_peak"] < 22.5
    assert both["Ts"] < 0.5 * base["Ts"]


def test_material_model_with_exact_parameters_tracks_the_plant():
    p = TsrdParams()
    log = run(batch_plan(5, 4, p, second_after=115.0, jitter=20.0), StartupConfig(), duration=500, seed=5,
              u_roll=u_roll_for(p, 1.0, P_SP))
    s = log.arr("t") > 100.0
    assert np.sqrt(np.mean((log.arr("M_hat")[s] - log.arr("M")[s]) ** 2)) < 1.0
    assert np.sqrt(np.mean((log.arr("m_s_hat")[s] - log.arr("m_s")[s]) ** 2)) < 1.0


def test_throughput_calibration_error_drifts_the_total_not_the_screw_register():
    p = TsrdParams()
    err = {"q_cal": 1.02}
    log = run(batch_plan(5, 4, p, second_after=115.0, jitter=20.0), StartupConfig(model_err=err), duration=500,
              seed=5, u_roll=u_roll_for(p, 1.0, P_SP))
    t, d = log.arr("t"), log.arr("M_hat") - log.arr("M")
    i200, i450 = int(np.argmin(abs(t - 200))), int(np.argmin(abs(t - 450)))
    # 2 % of the mass that has passed: the error grows with time and is negative
    assert d[i450] < d[i200] < 0
    assert d[i450] == pytest.approx(-0.02 * p.rho * p.Q_ref * 450, rel=0.25)
    s = t > 100.0
    assert np.sqrt(np.mean((log.arr("m_s_hat")[s] - log.arr("m_s")[s]) ** 2)) < 3.0


def test_pressure_above_threshold_pins_the_chamber_of_the_material_model():
    p = TsrdParams()
    mm = MaterialModel(p, {"q_cal": 1.05})
    mm.drop(220.0)
    for _ in range(3000):
        mm.step(12.0, p.Q_ref, 0.1, chamber_full=True)
    assert mm.v_ch == p.V_ch
