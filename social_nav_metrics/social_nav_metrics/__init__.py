from navigation_metrics.metric import nav_metric
from navigation_metrics.flexible_bag import flexible_bag_converter_function, BagMessage
from navigation_metrics.util import pose2d_distance

from social_nav_msgs.msg import Pedestrians


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
