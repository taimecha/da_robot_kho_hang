"""Simulation telemetry supervision, independent of ROS and GUI load."""
import math


class TelemetryGuard:
    """Warn on jitter; hold/recover on a gap; latch failure on lasting loss."""
    def __init__(self):
        self.paused_since = None
        self.fresh_since = None
        self.failed = False
        self.state = 'healthy'

    @property
    def paused(self):
        return self.paused_since is not None

    def step(self, now, ages):
        # ages: name -> (time since odometry source stamp, status source stamp)
        odom = max((v[0] for v in ages.values()), default=math.inf)
        status = max((v[1] for v in ages.values()), default=math.inf)
        if any(not math.isfinite(age) for values in ages.values() for age in values):
            odom = status = math.inf
        if self.failed or max(odom, status) >= 10 or (
                self.paused and now-self.paused_since >= 15):
            self.failed = True
            self.state = 'failed'
        elif self.paused:
            if odom < .3 and status < .8:
                if self.fresh_since is None:self.fresh_since = now
                if now-self.fresh_since >= 1:
                    self.paused_since = self.fresh_since = None
                    self.state = 'resumed'
                else:self.state = 'recovering'
            else:
                self.fresh_since = None
                self.state = 'paused'
        elif odom > 2 or status > 3:
            self.paused_since = now
            self.fresh_since = None
            self.state = 'paused'
        else:
            self.state = 'warning' if odom > 1 or status > 2 else 'healthy'
        return self.state
