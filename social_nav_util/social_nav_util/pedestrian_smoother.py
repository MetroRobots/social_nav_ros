from social_nav_msgs.msg import Pedestrians, Pedestrian
from nav_2d_msgs.msg import Twist2D
from math import atan2, hypot, cos, sin
from angles import shortest_angular_distance
from rclpy.time import Time
from rclpy.duration import Duration


class Velocity:
    def __init__(self, twist2d=None, angle=None, magnitude=None):
        self._twist2d = twist2d
        self._angle = angle
        self._magnitude = magnitude

    @property
    def x(self):
        if not self._twist2d:
            self._calculate_twist()
        return self._twist2d.x

    @property
    def y(self):
        if not self._twist2d:
            self._calculate_twist()
        return self._twist2d.y

    @property
    def angle(self):
        if not self._angle:
            self._calculate_polar()
        return self._angle

    @property
    def magnitude(self):
        if not self._angle:
            self._calculate_polar()
        return self._magnitude

    def _calculate_polar(self):
        if not self._twist2d:
            raise RuntimeError('Cannot calculate polar without twist2d')
        self._angle = atan2(self._twist2d.y, self._twist2d.x)
        self._magnitude = hypot(self._twist2d.y, self._twist2d.x)

    def _calculate_twist(self):
        if not self._angle or not self._magnitude:
            raise RuntimeError('Cannot calculate twist2d without polar coordinates')
        self._twist2d = Twist2D()
        self._twist2d.x = self._magnitude * cos(self._angle)
        self._twist2d.y = self._magnitude * sin(self._angle)

    def normalize_angle(self, other):
        if other is None:
            return
        delta = shortest_angular_distance(other.angle, self.angle)
        self._angle = other.angle + delta


class TimePoint:
    def __init__(self, pedestrians_msg):
        self.msg = pedestrians_msg
        self._velocity_map = {}
        self._t = None

    @property
    def timestamp(self):
        if not self._t:
            self._t = Time().from_msg(self.msg.header.stamp)
        return self._t

    @property
    def velocity_map(self):
        if not self._velocity_map:
            for ped in self.msg.pedestrians:
                self._velocity_map[ped.identifier] = Velocity(twist2d=ped.velocity)
        return self._velocity_map


class PedestrianSmoother:
    def __init__(self, window=5.0):
        self.window = Duration(seconds=window)
        self.buffer = []

    def update(self, pedestrians_msg):
        new_pt = TimePoint(pedestrians_msg)
        if self.buffer:
            prev_last = self.buffer[-1]
            for ped_id, v in new_pt.velocity_map.items():
                v.normalize_angle(prev_last.velocity_map.get(ped_id))
        self.buffer.append(new_pt)

        # Remove old
        while self.buffer[-1].timestamp - self.buffer[0].timestamp > self.window:
            self.buffer.pop(0)

    def get_velocity(self, key):
        velocities = self.get_velocities(key)
        # Return average
        n = len(velocities)
        velocity = Velocity(angle=sum(v.angle for v in velocities) / n,
                            magnitude=sum(v.magnitude for v in velocities) / n)
        return velocity

    def get_velocities(self, key):
        velocities = []
        for timepoint in self.buffer:
            if key in timepoint.velocity_map:
                velocities.append(timepoint.velocity_map[key])

        return velocities

    def get_smooth(self):
        if not self.buffer:
            return None

        last_msg = self.buffer[-1].msg

        new_msg = Pedestrians()
        new_msg.header = last_msg.header
        for last_ped in last_msg.pedestrians:
            new_ped = Pedestrian()
            new_ped.identifier = last_ped.identifier
            new_ped.pose = last_ped.pose
            new_ped.velocity = self.get_velocity(last_ped.identifier).twist2d

            new_msg.pedestrians.append(new_ped)
        return new_msg
