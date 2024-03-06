from navigation_metrics.metric import nav_metric, nav_metric_set
from navigation_metrics.flexible_bag import flexible_bag_converter_function, BagMessage
from navigation_metrics.util import pose2d_distance, min_max_avg_dev_d, metric_min, min_max_avg_d

from std_msgs.msg import Float64, Int32
from social_nav_msgs.msg import Pedestrians, PolarPedestrian, PolarPedestrians
from math import pi
import math


@flexible_bag_converter_function('/pedestrians', 'social_nav_msgs.msg.Pedestrians')
def convert_pedestrians(data):
    seq = []
    covar_topics = data.get_topics_by_type('social_nav_msgs/msg/PedestriansWithCovariance')
    if len(covar_topics) != 1:
        return []

    topic = covar_topics[0]
    for t, msg in data[topic]:
        nmsg = Pedestrians()
        nmsg.header = msg.header
        for ped_c in msg.pedestrians:
            nmsg.pedestrians.append(ped_c.pedestrian)
        seq.append(BagMessage(t, nmsg))
    return seq


@flexible_bag_converter_function('/polar_pedestrians', 'social_nav_msgs.msg.PolarPedestrians')
def convert_to_polar(data):
    robot_frame = data.get_parameter('robot_frame', 'base_link')
    seq = []
    for ped_rmsg, path_rmsg in data['/pedestrians', '/path2d']:
        msg = PolarPedestrians()
        msg.header.frame_id = robot_frame
        msg.header.stamp = ped_rmsg.msg.header.stamp

        for pedestrian in ped_rmsg.msg.pedestrians:
            d, angle = pose2d_distance(path_rmsg.msg.pose, pedestrian.pose)
            if math.isnan(d):
                continue
            pmsg = PolarPedestrian()
            pmsg.identifier = pedestrian.identifier
            pmsg.angle = angle
            pmsg.distance = d
            msg.pedestrians.append(pmsg)

        seq.append(BagMessage(ped_rmsg.t, msg))
    return seq


@flexible_bag_converter_function('/closest_person_distance')
def get_closest_person_distance(data):
    seq = []
    for t, msg in data['/polar_pedestrians']:
        min_d = float('inf')
        for pedestrian in msg.pedestrians:
            if pedestrian.distance < min_d:
                min_d = pedestrian.distance
        fmsg = Float64()
        fmsg.data = min_d
        seq.append(BagMessage(t, fmsg))
    return seq


@nav_metric
def min_distance_to_person(data):
    """
    The minimum distance to any person over the trial.

    Units: meters
    """
    return metric_min(data['/closest_person_distance'])


def calculate_density(msg, radius, angle):
    count = 0
    for pedestrian in msg.pedestrians:
        if pedestrian.distance < radius and pedestrian.angle < angle:
            count += 1
    return count


@flexible_bag_converter_function('/fov_count')
def calculate_fov_count(data):
    fov_density_radius = data.get_parameter('fov_density_radius', 3.0)
    fov_density_angle = data.get_parameter('fov_density_angle', pi / 4.0)
    seq = []
    for t, msg in data['/polar_pedestrians']:
        fmsg = Int32()
        fmsg.data = calculate_density(msg, fov_density_radius, fov_density_angle)
        seq.append(BagMessage(t, fmsg))
    return seq


@flexible_bag_converter_function('/pedestrian_count')
def calculate_pedestrian_count(data):
    seq = []
    for t, msg in data['/polar_pedestrians']:
        fmsg = Int32()
        fmsg.data = len(msg.pedestrians)
        seq.append(BagMessage(t, fmsg))
    return seq


def calculate_close_pedestrian_count(data, close_pedestrian_radius):
    seq = []
    for t, msg in data['/polar_pedestrians']:
        fmsg = Int32()
        for ped in msg.pedestrians:
            if ped.distance <= close_pedestrian_radius:
                fmsg.data += 1
        seq.append(BagMessage(t, fmsg))
    return seq


@nav_metric_set(['min', 'max', 'avg'])
def pedestrian_counts(data):
    """
    The minimum/maximum/average number of pedestrians detected

    Units: Count
    """
    return min_max_avg_d(data['/pedestrian_count'])


@nav_metric_set(['min', 'max', 'avg', 'stddev'])
def close_pedestrian_counts(data, close_pedestrian_radius=2.0):
    """
    The minimum/maximum/average and standard deviation of the number of pedestrians within the close_pedestrian_radius

    Units: Count
    """

    return min_max_avg_dev_d(calculate_close_pedestrian_count(data, close_pedestrian_radius))


@nav_metric_set(['count', 'density'])
def max_sustained_pedestrian(data, sustain_period=4.0, close_pedestrian_radius=2.0):
    """
    The greatest number of people sustained for at least the sustain_period

    The value is expressed both as an absolute count and as a density, i.e. the ratio of the count to the F.O.V. area

    Units: Count, Count / meter^2
    """
    fov_density_angle = data.get_parameter('fov_density_angle', pi / 4.0)
    fov_area = (fov_density_angle / 2) * close_pedestrian_radius * close_pedestrian_radius
    start_times = {}
    max_count = 0
    prev_t = None
    for t, msg in calculate_close_pedestrian_count(data, close_pedestrian_radius):
        count = msg.data
        if count not in start_times:
            start_times[count] = t
        else:
            for completed_count in sorted(start_times.keys()):
                if completed_count <= count:
                    continue
                start_t = start_times.pop(completed_count)
                delta = prev_t - start_t
                if delta > sustain_period and completed_count > max_count:
                    max_count = completed_count
        prev_t = t

    metrics = {}
    metrics['count'] = max_count
    metrics['density'] = max_count / fov_area
    return metrics


@nav_metric_set(['min', 'max', 'avg', 'stddev'])
def pedestrian_density(data):
    """
    The minimum and maximum, and average and standard deviation of the number of pedestrians detected.

    Split into the full count and that just in the field of view.

    Units: Count / meter^2
    """
    general_density_radius = data.get_parameter('general_density_radius', 10.0)
    fov_density_radius = data.get_parameter('fov_density_radius', 3.0)
    fov_density_angle = data.get_parameter('fov_density_angle', pi / 4.0)

    general_area = pi * general_density_radius * general_density_radius
    fov_area = (fov_density_angle / 2) * fov_density_radius * fov_density_radius

    metrics = {}
    for prefix, topic, denominator in [('', '/pedestrian_count', general_area),
                                       ('fov', '/fov_count', fov_area)]:
        d = min_max_avg_dev_d(data[topic])
        for k, v in d.items():
            if prefix:
                key = f'{prefix}/{k}'
            else:
                key = k
            metrics[key] = v / denominator
    return metrics


@nav_metric
def average_safety_distance(data, close_pedestrian_radius=2.0):
    min_distances = {}
    for t, msg in data['/polar_pedestrians']:
        for ped in msg.pedestrians:
            if math.isnan(ped.distance) or ped.distance > close_pedestrian_radius:
                continue
            key = ped.identifier
            if key not in min_distances or min_distances[key] > ped.distance:
                min_distances[key] = ped.distance

    if not min_distances:
        return

    V = min_distances.values()
    return sum(V) / len(V)


@nav_metric
def reciprocal_people_distance(data, reciprocal_people_distance_exponent=2):
    seq = []
    for t, msg in data['/polar_pedestrians']:
        total = 0.0
        for pedestrian in msg.pedestrians:
            total += math.pow(pedestrian.distance, -reciprocal_people_distance_exponent)
        fmsg = Float64()
        fmsg.data = total
        seq.append(BagMessage(t, fmsg))

    return min_max_avg_d(seq)
