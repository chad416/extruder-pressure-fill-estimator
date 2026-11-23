"""Zero check of the head-pressure transducer at run-out (FMEA row F3, slow zero drift).

A zero drift cannot be seen in production: the controller holds the *reading* at the
setpoint, and the fill estimator explains the reading with a slightly different filled
length (results/drift.md). The one moment the true pressure is known without a second
sensor is the end of a run-out, when the screw is empty and the head has drained. The
reading at that moment is the zero offset.

The check uses only signals the PLC already has (feed-on flag, drive torque, the pressure
reading) and no sample buffer, so it ports to SCL as two first-order filters and a timer:

1. Armed while the feed is off and the torque is below `torque_empty` (an empty screw turns
   against friction only).
2. A fast and a slow first-order filter follow the reading. While the head is still
   draining they disagree. When they agree within `band_bar` for `dwell_s`, the slow filter
   value is latched as the zero offset.

Validity limit: it assumes the head really drains to zero gauge pressure at run-out. The
simulator's head does, because the gear pump keeps turning. Whether a real head does, with
compound left between screw tip and pump, is a question for the plant (dossier §12, Q5).
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class ZeroCheckConfig:
    torque_empty: float = 4000.0  # N m; about 10 % of the rated torque of the 185 kW drive
    tau_fast: float = 2.0  # s
    tau_slow: float = 10.0  # s
    band_bar: float = 0.3  # the two filters must agree within this band
    dwell_s: float = 20.0  # ... for this long
    alarm_bar: float = 2.0  # |zero| above this raises the recalibration flag


class RunoutZeroCheck:
    def __init__(self, cfg: ZeroCheckConfig | None = None):
        self.cfg = cfg or ZeroCheckConfig()
        self.armed = False
        self.p_fast = 0.0
        self.p_slow = 0.0
        self.timer = 0.0
        self.zero_bar: float | None = None  # latched result of the last completed check
        self.t_capture: float | None = None  # time since arming at the capture, s
        self._t_armed = 0.0

    @property
    def alarm(self) -> bool:
        return self.zero_bar is not None and abs(self.zero_bar) > self.cfg.alarm_bar

    def step(self, feed_on: bool, torque: float, p_meas_bar: float, dt: float) -> float | None:
        """Call once per PLC cycle. Returns the zero offset in the cycle it is captured."""
        c = self.cfg
        if feed_on or torque > c.torque_empty:
            self.armed = False
            self.timer = 0.0
            return None
        if not self.armed:
            self.armed = True
            self.p_fast = self.p_slow = p_meas_bar
            self.timer = 0.0
            self._t_armed = 0.0
            return None
        self._t_armed += dt
        self.p_fast += (p_meas_bar - self.p_fast) * dt / c.tau_fast
        self.p_slow += (p_meas_bar - self.p_slow) * dt / c.tau_slow
        if abs(self.p_fast - self.p_slow) < c.band_bar:
            self.timer += dt
        else:
            self.timer = 0.0
        if self.timer >= c.dwell_s and self.t_capture is None:
            self.zero_bar = self.p_slow
            self.t_capture = self._t_armed
            return self.zero_bar
        return None

    def reset(self) -> None:
        """Clear the latched result (start of the next campaign)."""
        self.zero_bar = None
        self.t_capture = None
        self.armed = False
        self.timer = 0.0
