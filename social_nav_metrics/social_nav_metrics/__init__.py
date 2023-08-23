from navigation_metrics.metric import nav_metric
from navigation_metrics.flexible_bag import flexible_bag_converter_function, BagMessage
from navigation_metrics.util import pose2d_distance, min_max_total_avg

from social_nav_msgs.msg import Pedestrians
from math import pi


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


@nav_metric
def min_distance_to_person(data):
    min_d = None
    for ped_rmsg, path_rmsg in data['/pedestrians', '/path2d']:
        for pedestrian in ped_rmsg.msg.pedestrians:
            d, angle = pose2d_distance(path_rmsg.msg.pose, pedestrian.pose)
            if min_d is None or d < min_d:
                min_d = d
    return min_d


@nav_metric
def pedestrian_density(data):
    general_density_radius = data.get_parameter('general_density_radius', 10.0)
    fov_density_radius = data.get_parameter('fov_density_radius', 3.0)
    fov_density_angle = data.get_parameter('fov_density_angle', pi / 4.0)

    general_area = pi * general_density_radius * general_density_radius
    fov_area = (fov_density_angle / 2) * fov_density_radius * fov_density_radius
    general_densities = []
    fov_densities = []

    for ped_rmsg, path_rmsg in data['/pedestrians', '/path2d']:
        general_count = 0
        fov_count = 0
        for pedestrian in ped_rmsg.msg.pedestrians:
            d, angle = pose2d_distance(path_rmsg.msg.pose, pedestrian.pose)
            if d < general_density_radius:
                general_count += 1

            if d < fov_density_radius and abs(angle) < fov_density_angle:
                fov_count += 1

        general_densities.append(general_count / general_area)
        fov_densities.append(fov_count / fov_area)

    d = {}
    for prefix, values in [('', general_densities), ('fov_', fov_densities)]:
        the_min, the_max, _, avg = min_max_total_avg(values, lambda rmsg: rmsg)
        d[f'min_{prefix}pedestrian_density'] = the_min
        d[f'max_{prefix}pedestrian_density'] = the_max
        d[f'avg_{prefix}pedestrian_density'] = avg
    return d
