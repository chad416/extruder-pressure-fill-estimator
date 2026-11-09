"""Learned estimators of the compound left in the screw (the 'NN/AI' half of topic 3).

The features come from 1 Hz logs. Each signal is reduced to trailing means over 5, 20
and 60 s and to differences over 20 and 60 s. Two input sets are compared:
- "tpr": torque, power and rpm only, which is what topic 3 names;
- "full": the tpr signals plus head pressure, melt temperature and gear-pump speed.

The two models are:
- ridge regression;
- a small MLP: 2 x 24 tanh units, Adam, L2 penalty, early stop on a held-out split.

Both are written in numpy, so the repo needs no ML framework, and the trained MLP has
few enough weights (about 1.3 k) to port to SCL if it were ever chosen.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .sim import Log

SETS = {
    "tpr": ("torque", "power", "N"),
    "full": ("torque", "power", "N", "P_meas", "T_meas", "gp"),
}


def features(log: Log, which: str) -> tuple[np.ndarray, np.ndarray]:
    """Feature matrix (one row per logged second, after 60 s) and target M."""
    cols = []
    for name in SETS[which]:
        x = log.arr(name)
        c = np.cumsum(np.r_[0.0, x])
        for w in (5, 20, 60):
            m = np.empty_like(x)
            for i in range(len(x)):
                a = max(0, i - w + 1)
                m[i] = (c[i + 1] - c[a]) / (i + 1 - a)
            cols.append(m)
        for lag in (20, 60):
            d = np.zeros_like(x)
            d[lag:] = x[lag:] - x[:-lag]
            cols.append(d)
    X = np.column_stack(cols)[60:]
    y = log.arr("M")[60:]
    return X, y


@dataclass
class Scaler:
    mu: np.ndarray
    sd: np.ndarray

    @classmethod
    def fit(cls, X: np.ndarray) -> "Scaler":
        return cls(X.mean(0), X.std(0) + 1e-9)

    def __call__(self, X: np.ndarray) -> np.ndarray:
        return (X - self.mu) / self.sd


class Ridge:
    def __init__(self, lam: float = 1.0):
        self.lam = lam

    def fit(self, X: np.ndarray, y: np.ndarray) -> "Ridge":
        self.sx = Scaler.fit(X)
        self.ym = y.mean()
        Z = self.sx(X)
        A = Z.T @ Z + self.lam * np.eye(Z.shape[1])
        self.w = np.linalg.solve(A, Z.T @ (y - self.ym))
        return self

    def predict(self, X: np.ndarray) -> np.ndarray:
        return self.sx(X) @ self.w + self.ym


class MLP:
    def __init__(self, hidden: int = 24, l2: float = 1e-4, lr: float = 3e-3, epochs: int = 400,
                 seed: int = 0):
        self.h, self.l2, self.lr, self.epochs, self.seed = hidden, l2, lr, epochs, seed

    def _init(self, d: int) -> None:
        r = np.random.default_rng(self.seed)
        h = self.h
        self.W = [r.normal(0, 1 / np.sqrt(d), (d, h)), r.normal(0, 1 / np.sqrt(h), (h, h)),
                  r.normal(0, 1 / np.sqrt(h), (h, 1))]
        self.b = [np.zeros(h), np.zeros(h), np.zeros(1)]

    def _fwd(self, Z: np.ndarray):
        a1 = np.tanh(Z @ self.W[0] + self.b[0])
        a2 = np.tanh(a1 @ self.W[1] + self.b[1])
        return a1, a2, (a2 @ self.W[2] + self.b[2])[:, 0]

    def fit(self, X: np.ndarray, y: np.ndarray) -> "MLP":
        self.sx = Scaler.fit(X)
        self.ym, self.ys = y.mean(), y.std() + 1e-9
        Z = self.sx(X)
        t = (y - self.ym) / self.ys
        r = np.random.default_rng(self.seed)
        idx = r.permutation(len(Z))
        nv = len(Z) // 5
        va, tr = idx[:nv], idx[nv:]
        self._init(Z.shape[1])
        params = self.W + self.b
        m = [np.zeros_like(p) for p in params]
        v = [np.zeros_like(p) for p in params]
        best, best_p, bad = np.inf, None, 0
        bs = 512
        step = 0
        for ep in range(self.epochs):
            r.shuffle(tr)
            for k in range(0, len(tr), bs):
                bi = tr[k:k + bs]
                Zb, tb = Z[bi], t[bi]
                a1, a2, out = self._fwd(Zb)
                g = (out - tb)[:, None] * (2 / len(bi))
                gW2 = a2.T @ g
                gb2 = g.sum(0)
                d2 = (g @ self.W[2].T) * (1 - a2**2)
                gW1 = a1.T @ d2
                gb1 = d2.sum(0)
                d1 = (d2 @ self.W[1].T) * (1 - a1**2)
                gW0 = Zb.T @ d1
                gb0 = d1.sum(0)
                grads = [gW0 + self.l2 * self.W[0], gW1 + self.l2 * self.W[1], gW2 + self.l2 * self.W[2],
                         gb0, gb1, gb2]
                step += 1
                for i, (p, gi) in enumerate(zip(params, grads)):
                    m[i] = 0.9 * m[i] + 0.1 * gi
                    v[i] = 0.999 * v[i] + 0.001 * gi * gi
                    mh = m[i] / (1 - 0.9**step)
                    vh = v[i] / (1 - 0.999**step)
                    p -= self.lr * mh / (np.sqrt(vh) + 1e-8)
            _, _, ov = self._fwd(Z[va])
            loss = float(np.mean((ov - t[va]) ** 2))
            if loss < best - 1e-5:
                best, bad = loss, 0
                best_p = [p.copy() for p in params]
            else:
                bad += 1
                if bad > 30:
                    break
        if best_p is not None:
            for p, q in zip(params, best_p):
                p[...] = q
        return self

    def predict(self, X: np.ndarray) -> np.ndarray:
        return self._fwd(self.sx(X))[2] * self.ys + self.ym
