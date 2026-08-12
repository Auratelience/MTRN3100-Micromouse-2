"""Mirrors the public surface of micromouse/lidar.h.

Only the *observable behaviour* of the VL6180X is modelled -- continuous
ranging period, 2 mm quantisation at X2 scaling, and the min/max clamps that
the firmware's read() applies on the range-status error codes. The I2C
register plumbing, LidarSensor::status()/exists()/recover() and the VL6180X
driver itself have no meaning in simulation and are not ported.
"""

from .constants import LIDAR_CONTINUOUS_PERIOD_MS


class L:
    """Mirrors L::ReadingConstants."""

    MIN_DIST = 0
    MAX_DIST = 300

    # Scaling X2: 2 mm resolution, 400 mm nominal ceiling. The firmware's
    # error handling clamps to MAX_DIST well before that.
    SCALE_MM = 2


class LIDAR:
    Front = 0
    Left = 1
    Right = 2
    COUNT = 3

    _NAMES = {Front: "front", Left: "left", Right: "right"}

    def __init__(self, plant):
        self.plant = plant
        self.readings = [0, 0, 0]
        self._last_sample = [None, None, None]
        self.period_s = LIDAR_CONTINUOUS_PERIOD_MS / 1000.0
        # Stands in for the millis() the free-running sensor is measured
        # against. Runner advances it once per control loop; LidarObserver then
        # calls update() with no argument, exactly as the firmware does.
        self.t = 0.0

    def init(self):
        return True

    def advance(self, dt):
        """Move the sensor's own clock on by dt seconds. Sim-only -- on
        hardware this is millis() ticking underneath."""
        self.t += dt

    def update(self, now_s=None):
        """Re-read any sensor whose continuous-ranging period has elapsed.

        Between periods the previous value is held, exactly as reading the
        result register between conversions does on hardware. A control loop
        running at 1 kHz therefore sees the same reading for ten iterations.

        `now_s` defaults to the internal clock, which is what the firmware's
        argument-free LIDAR::update() amounts to.
        """
        if now_s is None:
            now_s = self.t
        for i in range(self.COUNT):
            last = self._last_sample[i]
            if last is not None and (now_s - last) < self.period_s:
                continue
            self.readings[i] = self._sample(self._NAMES[i])
            self._last_sample[i] = now_s

    def getReading(self, sensor):
        return self.readings[sensor]

    def reset(self):
        self.readings = [0, 0, 0]
        self._last_sample = [None, None, None]
        self.t = 0.0

    def _sample(self, name):
        d = self.plant.range_mm(name)
        if d <= 0.0:
            return L.MIN_DIST
        if d >= L.MAX_DIST:
            return L.MAX_DIST
        quantised = int(d / L.SCALE_MM) * L.SCALE_MM
        if quantised > L.MAX_DIST:
            return L.MAX_DIST
        return quantised
