import numpy as np
from scipy.signal import lfilter

from xtrude.control import (PIDRuntime, RSTRuntime, design_rst, padd, pid_as_rst, pmul,
                            solve_diophantine)
from xtrude.ident import Model

M = Model(np.array([1.0, -2.5225, 2.1017, -0.5787]), np.array([0.0, 0.3232, -0.4223, 0.1121]))


def test_diophantine_identity():
    Ap = pmul(M.a, np.array([1.0, -1.0]))
    P = pmul(np.array([1, -0.9]), np.array([1, -0.8]), np.array([1, -0.5]), np.array([1, -0.5]),
             np.array([1, -0.4]), np.array([1, -0.3]))
    S, R = solve_diophantine(Ap, M.b, P)
    lhs = padd(pmul(Ap, S), pmul(M.b, R))
    assert np.allclose(lhs[:len(P)], P, atol=1e-10)
    assert np.allclose(lhs[len(P):], 0, atol=1e-10)


def test_rst_has_integral_action_and_unit_static_gain():
    c = design_rst(M, w0=0.7, zeta=0.8, aux=0.6)
    assert abs(np.sum(c.S)) < 1e-9  # S(1) = 0
    assert abs(np.sum(c.T) - np.sum(c.R)) < 1e-6  # T(1) = R(1)
    Pc = padd(pmul(M.a, c.S), pmul(M.b, c.R))
    assert np.all(np.abs(np.roots(Pc)) < 1)


def test_rst_runtime_equals_linear_law_without_saturation():
    c = design_rst(M, w0=0.7, zeta=0.8, aux=0.6)
    rt = RSTRuntime(c, -1e9, 1e9, 1e9)
    rt.init(0.0, 0.0, 0.0)
    rng = np.random.default_rng(0)
    r = np.r_[np.zeros(20), np.ones(300)]
    y = rng.normal(0, 0.3, len(r))
    u = np.array([rt.step(r[k], y[k]) for k in range(len(r))])
    rs = lfilter(c.Bm, c.Am, r)
    # S u = T r* - R y
    lhs = lfilter(c.S, [1.0], u)
    rhs = lfilter(c.T, [1.0], rs) - lfilter(c.R, [1.0], y)
    assert np.max(np.abs(lhs - rhs)) < 1e-8


def test_pid_runtime_matches_rst_form():
    Kp, Ti, Td = 0.11, 0.49, 0.17
    pid = PIDRuntime(Kp, Ti, Td, -1e9, 1e9, 1e9)
    pid.init(0.0, 0.0, 0.0)
    c = pid_as_rst(Kp, Ti, Td)
    rng = np.random.default_rng(1)
    y = rng.normal(0, 1, 400)
    r = np.zeros(400)
    u = np.array([pid.step(r[k], y[k]) for k in range(400)])
    u2 = lfilter(-c.R, c.S, y)
    assert np.max(np.abs(u - u2)) < 1e-9


def test_antiwindup_recovers():
    c = design_rst(M, w0=0.7, zeta=0.8, aux=0.6)
    rt = RSTRuntime(c, 25.0, 35.0, 0.5)
    rt.init(60.0, 60.0, 30.0)
    # long large error: output saturates, then the error vanishes
    for _ in range(300):
        u = rt.step(60.0, 40.0)
    assert u == 35.0
    for _ in range(50):
        u = rt.step(60.0, 60.0)
    assert u < 35.0  # leaves the limit promptly (no integrator wind-up)
