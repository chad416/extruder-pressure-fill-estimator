"""The SCL that goes into TIA Portal runs here through the translator (float32 Real)."""

from pathlib import Path

import numpy as np
import pytest

from xtrude import scl
from xtrude.control import PIDRuntime, RSTRuntime, design_rst
from xtrude.estimator import FillEKF
from xtrude.ident import TS, Model
from xtrude.plant import nominal

ROOT = Path(__file__).resolve().parents[1]
FB = scl.load(sorted(ROOT.glob("plc/FB_*.scl")))
M = Model(np.array([1.0, -2.5225, 2.1017, -0.5787]), np.array([0.0, 0.3232, -0.4223, 0.1121]))
f32 = np.float32


def _closed_loop(step, n=2500):
    """Linear plant (deviation from 60 bar / 30 rpm) with noise and an input disturbance."""
    rng = np.random.default_rng(0)
    a, b = M.a, M.b
    yh, uh = np.zeros(len(a)), np.zeros(len(b))
    u, U, Y = 30.0, [], []
    for k in range(n):
        sp = 60.0 if k < 1200 else 64.0
        d = 1.5 if 400 < k < 800 else 0.0
        uh = np.roll(uh, 1)
        uh[0] = u - 30.0 + d
        yd = float(np.dot(b, uh) - np.dot(a[1:], yh[:-1]))
        yh = np.roll(yh, 1)
        yh[0] = yd
        y = 60.0 + yd + rng.normal(0, 0.3)
        u = step(sp, y)
        U.append(u)
        Y.append(y)
    return np.array(U), np.array(Y)


def test_pid_scl_matches_reference():
    ref = PIDRuntime(0.11, 0.49, 0.17, 2, 45, 0.5, Tt=0.49)
    ref.init(60, 60, 30)
    s = FB["FB_PressurePID"]()
    s.Kp, s.Ti, s.Td, s.Tt = f32(0.11), f32(0.49), f32(0.17), f32(0.49)
    s.Enable, s.Setpoint, s.Pv, s.TrackValue = False, f32(60), f32(60), f32(30)
    s()
    s.Enable = True

    def st(sp, y):
        s.Setpoint, s.Pv = f32(sp), f32(y)
        s()
        return float(s.Out)

    U1, _ = _closed_loop(lambda sp, y: ref.step(sp, y))
    U2, _ = _closed_loop(st)
    assert np.max(np.abs(U1 - U2)) < 1e-3


def _load_rst(r, c):
    for name in ("R", "S", "T", "Ao", "Bm", "Am"):
        arr, vals = getattr(r, name), getattr(c, name)
        for i in range(len(arr)):
            arr[i] = f32(vals[i] if i < len(vals) else 0.0)
    r.nR, r.nS, r.nT, r.nAo = len(c.R) - 1, len(c.S) - 1, len(c.T) - 1, len(c.Ao) - 1


def test_rst_scl_matches_reference_in_closed_loop():
    c = design_rst(M, w0=0.72, zeta=0.6, aux=0.755, wm=1.5)
    ref = RSTRuntime(c, 2, 45, 0.5)
    ref.init(60, 60, 30)
    r = FB["FB_PressureRST"]()
    _load_rst(r, c)
    r.Enable, r.Setpoint, r.Pv, r.TrackValue = False, f32(60), f32(60), f32(30)
    r()
    r.Enable = True

    def st(sp, y):
        r.Setpoint, r.Pv = f32(sp), f32(y)
        r()
        return float(r.Out)

    U1, Y1 = _closed_loop(lambda sp, y: ref.step(sp, y))
    U2, Y2 = _closed_loop(st)
    assert np.max(np.abs(U1 - U2)) < 0.01
    assert abs(Y2[-300:].mean() - 64.0) < 0.1  # no static offset in Real arithmetic


def test_ekf_scl_matches_reference():
    e = nominal(seed=4)
    ekf = FillEKF(e)
    s = FB["FB_FillEKF"]()
    pid = PIDRuntime(0.11, 0.49, 0.17, 2, 45, 0.5)
    pid.init(60, 60, 30)
    diffs, errs = [], []
    for k in range(2000):
        if k == 300:
            e.eps = 0.92
        bev = k == 1000
        if bev:
            e.kappa = 1.2
        m = e.measure()
        if k % 5 == 0:
            ekf.step(m, e.N_gp_rel, e.feed_on, bev)
            for j in range(5):  # same signals, so the 5th call runs the filter step
                s.Nrpm, s.Tmelt, s.Pbar, s.Torque = f32(m["N_rpm"]), f32(m["T"]), f32(m["P_bar"]), f32(m["torque"])
                s.GpRel, s.FeedOn, s.BatchEvent, s.PSp = f32(e.N_gp_rel), e.feed_on, bev and j == 0, f32(60)
                s()
            diffs.append(abs(ekf.mass() - float(s.Mass)))
            errs.append(abs(ekf.mass() - e.mass()))
        e.step(pid.step(60, m["P_bar"]), TS)
    assert max(diffs) < 0.02  # kg, float32 vs float64
    assert np.mean(errs[100:]) < 0.3


def test_supervisor_fallbacks():
    s = FB["FB_PressureSupervisor"]()
    s.ModeRequest, s.PChannelOk, s.TorqueOk, s.EkfValid = 3, True, True, True
    s.Pbar, s.Torque, s.RstOut, s.PidOut = f32(60), f32(25000), f32(30), f32(29)
    s()
    assert s.ActiveMode == 3 and s.FfEnable
    s.TorqueOk = False
    s()
    assert s.ActiveMode == 2 and not s.FfEnable
    s.PChannelOk = False
    s()
    assert s.ActiveMode == 4 and s.PFault
    s.PChannelOk = True
    s()
    assert s.ActiveMode == 4  # latched until acknowledged
    s.Ack = True
    s()
    s.Ack = False
    s.Pbar = f32(160)
    s()
    assert s.ActiveMode == 5 and float(s.SpeedSp) < 30


def test_frozen_pressure_detected():
    s = FB["FB_PressureSupervisor"]()
    s.ModeRequest, s.PChannelOk, s.TorqueOk, s.EkfValid = 2, True, True, True
    s.Pbar, s.Torque = f32(60), f32(25000)
    for k in range(260):
        s.RstOut = f32(30 + 0.01 * k)  # speed moves 2.6 rpm, pressure frozen
        s()
    assert s.PFault and s.ActiveMode == 4


def test_op_counter_counts():
    r = FB["FB_PressureRST"]()
    _load_rst(r, design_rst(M, w0=0.72, zeta=0.6, aux=0.755, wm=1.5))
    r.Enable = True
    scl.reset_ops()
    r()
    assert scl.OPS["real"] > 30 and scl.OPS["mov"] > 10


def test_reserved_and_unsupported_rejected():
    with pytest.raises(scl.SclError):
        scl.translate(['FUNCTION_BLOCK "X"\nVAR_OUTPUT\nTime : Real;\nEND_VAR\nBEGIN\nEND_FUNCTION_BLOCK'])
    with pytest.raises(scl.SclError):
        scl.translate(['FUNCTION_BLOCK "X"\nVAR\na : Real;\nEND_VAR\nBEGIN\n#a := SIN(1.0);\nEND_FUNCTION_BLOCK'])
