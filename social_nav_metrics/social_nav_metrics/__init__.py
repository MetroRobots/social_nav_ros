from navigation_metrics.metric import nav_metric
from navigation_metrics.flexible_bag import flexible_bag_converter_function, BagMessage
from navigation_metrics.util import pose2d_distance, min_max_total_avg, metric_min

from std_msgs.msg import Float64, Int32
from social_nav_msgs.msg import Pedestrians, PolarPedestrian, PolarPedestrians
from math import pi
import math


@flexible_bag_converter_function('/pedestrians')
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


@flexible_bag_converter_function('/polar_pedestrians')
def convert_to_polar(data):
    robot_frame = data.get_parameter('robot_frame', 'base_link')
    seq = []
    for ped_rmsg, path_rmsg in data['/pedestrians', '/path2d']:
        msg = PolarPedestrians()
        msg.header.frame_id = robot_frame
        msg.header.stamp = ped_rmsg.msg.header.stamp

        for pedestrian in ped_rmsg.msg.pedestrians:
            d, angle = pose2d_distance(path_rmsg.msg.pose, pedestrian.pose)
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


@flexible_bag_converter_function('/close_pedestrian_count')
def calculate_close_pedestrian_count(data):
    close_pedestrian_radius = data.get_parameter('close_pedestrian_radius', 2.0)
    seq = []
    for t, msg in data['/polar_pedestrians']:
        fmsg = Int32()
        for ped in msg.pedestrians:
            if ped.distance <= close_pedestrian_radius:
                fmsg.data += 1
        seq.append(BagMessage(t, fmsg))
    return seq


@nav_metric
def pedestrian_counts(data):
    d = {}
    the_min, the_max, _, avg = min_max_total_avg(data['/pedestrian_count'])
    d[f'min_pedestrian_count'] = the_min
    d[f'max_pedestrian_count'] = the_max
    d[f'avg_pedestrian_count'] = avg
    return d



@nav_metric
def close_pedestrian_counts(data):
    d = {}
    the_min, the_max, _, avg = min_max_total_avg(data['/close_pedestrian_count'])
    d[f'min_close_pedestrian_count'] = the_min
    d[f'max_close_pedestrian_count'] = the_max
    d[f'avg_close_pedestrian_count'] = avg
    return d

@nav_metric
def pedestrian_density(data):
    general_density_radius = data.get_parameter('general_density_radius', 10.0)
    fov_density_radius = data.get_parameter('fov_density_radius', 3.0)
    fov_density_angle = data.get_parameter('fov_density_angle', pi / 4.0)

    general_area = pi * general_density_radius * general_density_radius
    fov_area = (fov_density_angle / 2) * fov_density_radius * fov_density_radius

    d = {}
    for prefix, topic, denominator in [('', '/pedestrian_count', general_area),
                                       ('fov_', '/fov_count', fov_area)]:
        the_min, the_max, _, avg = min_max_total_avg(data[topic])
        d[f'min_{prefix}pedestrian_density'] = the_min / denominator
        d[f'max_{prefix}pedestrian_density'] = the_max / denominator
        d[f'avg_{prefix}pedestrian_density'] = avg / denominator
    return d


@flexible_bag_converter_function('/reciprocal_people_distance')
def reciprocal_people_distance(data):
    seq = []
    exp = data.get_parameter('reciprocal_people_distance_exponent', 2)
    for t, msg in data['/polar_pedestrians']:
        total = 0.0
        for pedestrian in msg.pedestrians:
            total += math.pow(pedestrian.distance, -exp)
        fmsg = Float64()
        fmsg.data = total
        seq.append(BagMessage(t, fmsg))
    return seq

@nav_metric
def rpd_metric(data):
    d = {}
    the_min, the_max, _, avg = min_max_total_avg(data['/reciprocal_people_distance'])
    d[f'min_rpd'] = the_min
    d[f'max_rpd'] = the_max
    d[f'avg_rpd'] = avg
    return d