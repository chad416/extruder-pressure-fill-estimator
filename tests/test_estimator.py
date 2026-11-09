import numpy as np

from xtrude.control import PIDRuntime
from xtrude.estimator import FillEKF, static_mass_balance
from xtrude.ident import TS
from xtrude.learned import MLP, Ridge
from xtrude.plant import nominal
from xtrude.sim import Event, Scenario, run


def test_static_mass_balance_at_steady_state():
    e = nominal()
    tr = e.truth()
    mb = static_mass_balance(e, tr["N_rpm"], tr["P_bar"], e.N_gp_rel)
    assert abs(mb - tr["M"]) / tr["M"] < 0.03


def test_ekf_tracks_mass_through_feed_change_and_runout():
    scn = Scenario(400.0, [Event(60, "eps", 0.92), Event(300, "feed_off")])
    log = run(scn, PIDRuntime(0.11, 0.49, 0.17, 2, 45, 0.5), use_ekf=True, plant_seed=3)
    t, M, Mh, sig = log.arr("t"), log.arr("M"), log.arr("M_hat"), log.arr("M_sig")
    s = t > 60
    assert np.sqrt(np.mean((Mh[s] - M[s]) ** 2)) < 0.3
    assert np.all(sig[s] > 0)
    assert M[-1] < M[200] - 1.0  # run-out empties the screw


def test_feedforward_reduces_pressure_deviation_for_feed_steps():
    scn = Scenario(500.0, [Event(60, "eps", 0.92), Event(260, "eps", 1.0)])
    base = run(scn, PIDRuntime(0.11, 0.49, 0.17, 2, 45, 0.5), plant_seed=5)
    ff = run(scn, PIDRuntime(0.11, 0.49, 0.17, 2, 45, 0.5), feedforward=True, plant_seed=5)

    def dev(log):
        s = log.arr("t") > 60
        return np.percentile(np.abs(log.arr("P")[s] - 60.0), 99)

    assert dev(ff) < dev(base)


def test_learned_models_fit_simple_function():
    rng = np.random.default_rng(0)
    X = rng.normal(size=(2000, 4))
    y = 2 * X[:, 0] - X[:, 1] + 0.5 * np.tanh(X[:, 2]) + 0.01 * rng.normal(size=2000)
    assert np.sqrt(np.mean((Ridge(1e-3).fit(X, y).predict(X) - y) ** 2)) < 0.2
    assert np.sqrt(np.mean((MLP(epochs=200).fit(X, y).predict(X) - y) ** 2)) < 0.1
