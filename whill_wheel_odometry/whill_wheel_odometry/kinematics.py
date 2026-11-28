"""ROS-independent differential drive model (SI units)."""
from dataclasses import dataclass
from math import atan2, cos, isfinite, sin


def _transpose(a):
    return [list(row) for row in zip(*a)]


def _multiply(a, b):
    return [[sum(x * y for x, y in zip(row, col)) for col in zip(*b)] for row in a]


def _add(a, b):
    return [[x + y for x, y in zip(ar, br)] for ar, br in zip(a, b)]


def _propagate(j, p):
    return _multiply(_multiply(j, p), _transpose(j))


def _diag(*values):
    return [[v if i == j else 0.0 for j in range(len(values))] for i, v in enumerate(values)]


@dataclass(frozen=True)
class DriveModel:
    wheel_tread: float = 0.496
    left_sign: int = 1
    right_sign: int = 1
    left_scale: float = 1.0
    right_scale: float = 1.0
    axle_to_base_x: float = 0.0
    axle_to_base_y: float = 0.0
    wheel_speed_std: float = 0.03
    lateral_speed_std: float = 0.05
    max_wheel_speed: float = 2.0

    def __post_init__(self):
        values = (self.wheel_tread, self.left_scale, self.right_scale,
                  self.axle_to_base_x, self.axle_to_base_y, self.wheel_speed_std,
                  self.lateral_speed_std, self.max_wheel_speed)
        if not all(isfinite(v) for v in values):
            raise ValueError('All model parameters must be finite')
        if self.left_sign not in (-1, 1) or self.right_sign not in (-1, 1):
            raise ValueError('Wheel signs must be +1 or -1')
        if min(self.wheel_tread, self.left_scale, self.right_scale, self.wheel_speed_std,
               self.lateral_speed_std, self.max_wheel_speed) <= 0.0:
            raise ValueError('Tread, scales, standard deviations and speed limit must be positive')

    def wheel_speeds(self, left_kmh, right_kmh):
        left = left_kmh * self.left_sign * self.left_scale / 3.6
        right = right_kmh * self.right_sign * self.right_scale / 3.6
        if not all(isfinite(v) and abs(v) <= self.max_wheel_speed for v in (left, right)):
            raise ValueError('Nonfinite or out-of-range wheel velocity')
        return left, right

    def axle_twist(self, left, right):
        return (left + right) / 2.0, (right - left) / self.wheel_tread

    def base_twist(self, left, right):
        v, w = self.axle_twist(left, right)
        return v - w * self.axle_to_base_y, w * self.axle_to_base_x, w

    def twist_covariance(self):
        # Map independent left/right speed noise and lateral slip to base twist.
        b, x, y = self.wheel_tread, self.axle_to_base_x, self.axle_to_base_y
        j = [[0.5 + y/b, 0.5 - y/b, 0.0],
             [-x/b, x/b, 1.0], [-1.0/b, 1.0/b, 0.0]]
        return _propagate(j, _diag(self.wheel_speed_std**2, self.wheel_speed_std**2,
                                   self.lateral_speed_std**2))


class WheelOdometry:
    def __init__(self, model):
        self.model = model
        # Initialize base_link at (0, 0, 0), not the axle center.
        self.x = -model.axle_to_base_x
        self.y = -model.axle_to_base_y
        self.yaw = 0.0
        self.p = _diag(1e-6, 1e-6, 1e-6)
        self.pose_valid = True

    def invalidate_pose(self):
        # A sample gap loses an unknown increment. Never claim full pose again
        # until a new origin is explicitly established by restarting the node.
        self.pose_valid = False

    def integrate(self, left, right, dt):
        if not isfinite(dt) or dt <= 0.0:
            raise ValueError('Integration dt must be positive and finite')
        v, w = self.model.axle_twist(left, right)
        a = w * dt / 2.0
        # Exact constant-twist displacement, including the straight-line limit.
        if abs(a) < 1e-5:
            s = 1.0 - a*a/6.0
            ds = -a/3.0 + a*a*a/30.0
        else:
            s = sin(a) / a
            ds = (a*cos(a) - sin(a)) / (a*a)
        h = self.yaw + a
        c, sn = cos(h), sin(h)
        d = v * dt * s
        dx, dy = d*c, d*sn
        f = [[1.0, 0.0, -dy], [0.0, 1.0, dx], [0.0, 0.0, 1.0]]
        dv = dt*s
        dw = v*dt*ds*dt/2.0
        g = [[dv*c, dw*c-d*sn*dt/2.0, -sn*dt],
             [dv*sn, dw*sn+d*c*dt/2.0, c*dt], [0.0, dt, 0.0]]
        b = self.model.wheel_tread
        j = [[0.5, 0.5, 0.0], [-1.0/b, 1.0/b, 0.0], [0.0, 0.0, 1.0]]
        q = _propagate(j, _diag(self.model.wheel_speed_std**2,
                                self.model.wheel_speed_std**2, self.model.lateral_speed_std**2))
        self.p = _add(_propagate(f, self.p), _propagate(g, q))
        self.x += dx
        self.y += dy
        self.yaw = atan2(sin(self.yaw + w*dt), cos(self.yaw + w*dt))

    def base_pose(self):
        x, y = self.model.axle_to_base_x, self.model.axle_to_base_y
        c, s = cos(self.yaw), sin(self.yaw)
        return self.x + c*x - s*y, self.y + s*x + c*y, self.yaw

    def pose_covariance(self):
        if not self.pose_valid:
            return _diag(1e6, 1e6, 1e6)
        x, y = self.model.axle_to_base_x, self.model.axle_to_base_y
        c, s = cos(self.yaw), sin(self.yaw)
        j = [[1.0, 0.0, -s*x-c*y], [0.0, 1.0, c*x-s*y], [0.0, 0.0, 1.0]]
        return _propagate(j, self.p)


def covariance6(planar, unknown=1e6):
    """Embed x, y, yaw (or vx, vy, wz) covariance into ROS 6x6 layout."""
    result = [0.0] * 36
    for i in (2, 3, 4):
        result[6*i+i] = unknown
    for i, row in enumerate((0, 1, 5)):
        for j, col in enumerate((0, 1, 5)):
            result[6*row+col] = planar[i][j]
    return result
