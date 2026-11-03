"""Latest camera frame and speed-adaptive joint smoothing; no device commands."""
import threading
import time

import numpy as np


class LatestFrameCapture:
    """One capture owner, one replaceable frame, no processing backlog.

    Timestamps are read-completion times, not camera exposure times. Consumers
    receive only a newer frame. A stopped camera cannot replay its last image.
    """
    def __init__(self, capture):
        self.capture = capture
        self.condition = threading.Condition()
        self.stopping = threading.Event()
        self.sequence = 0
        self.consumed = 0
        self.latest = None
        self.error = None
        self.thread = threading.Thread(target=self._capture, name='lh20-camera', daemon=True)
        self.thread.start()

    def _capture(self):
        try:
            while not self.stopping.is_set():
                ok, frame = self.capture.read()
                captured_ns = time.monotonic_ns()
                if not ok:
                    raise RuntimeError('Camera frame read failed')
                with self.condition:
                    if self.stopping.is_set():
                        break
                    self.sequence += 1
                    self.latest = (frame, captured_ns, self.sequence)
                    self.condition.notify_all()
        except Exception as error:
            with self.condition:
                self.error = error
                self.condition.notify_all()
        finally:
            self.capture.release()

    def read(self, timeout=2.):
        with self.condition:
            ready = self.condition.wait_for(
                lambda: self.error is not None or self.stopping.is_set() or self.sequence > self.consumed,
                timeout=timeout)
            if self.error is not None:
                raise RuntimeError('Camera capture stopped') from self.error
            if not ready or self.stopping.is_set():
                raise RuntimeError('No fresh camera frame')
            self.consumed = self.sequence
            return self.latest

    def close(self):
        self.stopping.set()
        with self.condition:
            self.condition.notify_all()
        # Only the capture thread calls release; avoid releasing a backend
        # concurrently with read. A backend stuck in read remains daemon-owned.
        self.thread.join(timeout=1.)


class AdaptiveJointFilter:
    """One-Euro-style speed adaptation, anchored to the old EMA at rest.

    The resting alpha stays 0.6 at every frame rate. The derivative uses actual
    acquisition intervals; fast motion raises the cutoff to reduce smoothing
    lag. This is a filter, not a prediction or a replacement for motor limits.
    """
    def __init__(self, adaptor=None, base_alpha=.6, beta=4., derivative_cutoff=1.):
        self.adaptor = adaptor
        self.base_alpha = base_alpha
        self.beta = beta
        self.derivative_cutoff = derivative_cutoff
        self.timestamp = None
        self.reset()

    def reset(self):
        self.previous_time = None
        self.previous_input = None
        self.derivative = None
        self.y = None

    def set_timestamp(self, timestamp):
        if not np.isfinite(timestamp):
            raise ValueError('Invalid filter timestamp')
        self.timestamp = float(timestamp)

    def next(self, x):
        x = np.asarray(x, dtype=float)
        if not np.isfinite(x).all():
            raise ValueError('Non-finite joint filter input')
        now = self.timestamp if self.timestamp is not None else time.monotonic()
        self.timestamp = None
        dt = None if self.previous_time is None else now - self.previous_time
        if dt is not None and dt <= 0:
            raise ValueError('Joint filter needs increasing frame timestamps')
        if self.y is None or dt > .25:
            self.y = x.copy()
            self.derivative = np.zeros_like(x)
        else:
            speed = (x - self.previous_input) / dt
            derivative_alpha = 1. / (1. + 1. / (2 * np.pi * self.derivative_cutoff * dt))
            self.derivative += derivative_alpha * (speed - self.derivative)
            base_cutoff = self.base_alpha / ((1 - self.base_alpha) * 2 * np.pi * dt)
            cutoff = base_cutoff + self.beta * np.abs(self.derivative)
            alpha = 1. / (1. + 1. / (2 * np.pi * cutoff * dt))
            self.y += alpha * (x - self.y)
        self.previous_time = now
        self.previous_input = x.copy()
        # Filtering mimics independently would slightly change their coupling
        # because their derivatives have different scales. Reapply exact mimic.
        if self.adaptor is not None:
            self.y = self.adaptor.forward_qpos(self.y.copy())
        return self.y.copy()
