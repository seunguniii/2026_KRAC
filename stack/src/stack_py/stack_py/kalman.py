import time
import numpy as np
import cv2
import math

from typing import Optional, Tuple

class TargetKalman2D:
    """
    2D target Kalman filter.

    State:
        x = [x_m, y_m, vx_mps, vy_mps]^T

    Measurement:
        z = [raw_x_m, raw_y_m]^T
    """

    def __init__(
        self,
        process_var: float = 0.01,
        measurement_var: float = 0.08,
        default_dt: float = 1.0 / 30.0,
    ) -> None:
        self.kf = cv2.KalmanFilter(4, 2)

        self.default_dt = default_dt
        self.initialized = False
        self.last_time = time.monotonic()

        self.kf.transitionMatrix = np.eye(4, dtype=np.float32)

        self.kf.measurementMatrix = np.array(
            [
                [1, 0, 0, 0],
                [0, 1, 0, 0],
            ],
            dtype=np.float32,
        )

        self.kf.processNoiseCov = np.array(
            [
                [process_var, 0, 0, 0],
                [0, process_var, 0, 0],
                [0, 0, process_var * 10.0, 0],
                [0, 0, 0, process_var * 10.0],
            ],
            dtype=np.float32,
        )

        self.kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * measurement_var
        self.kf.errorCovPost = np.eye(4, dtype=np.float32)

    def reset(self) -> None:
        self.initialized = False
        self.last_time = time.monotonic()

    def _get_dt(self) -> float:
        now = time.monotonic()
        dt = now - self.last_time
        self.last_time = now

        if dt <= 0.001 or dt > 1.0:
            dt = self.default_dt

        return dt

    def _update_transition(self, dt: float) -> None:
        self.kf.transitionMatrix = np.array(
            [
                [1, 0, dt, 0],
                [0, 1, 0, dt],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ],
            dtype=np.float32,
        )

    def update(self, raw_x_m: float, raw_y_m: float) -> Tuple[float, float]:
        dt = self._get_dt()
        self._update_transition(dt)

        if not math.isfinite(raw_x_m) or not math.isfinite(raw_y_m):
            return self.predict_only()

        if not self.initialized:
            self.kf.statePost = np.array(
                [
                    [raw_x_m],
                    [raw_y_m],
                    [0.0],
                    [0.0],
                ],
                dtype=np.float32,
            )
            self.initialized = True
            return raw_x_m, raw_y_m

        self.kf.predict()

        measurement = np.array(
            [
                [raw_x_m],
                [raw_y_m],
            ],
            dtype=np.float32,
        )

        estimated = self.kf.correct(measurement)

        return float(estimated[0, 0]), float(estimated[1, 0])

    def predict_only(self) -> Tuple[float, float]:
        if not self.initialized:
            return float("nan"), float("nan")

        dt = self._get_dt()
        self._update_transition(dt)

        predicted = self.kf.predict()

        return float(predicted[0, 0]), float(predicted[1, 0])
