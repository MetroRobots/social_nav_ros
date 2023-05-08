from rosbag2_py import SequentialReader
from rosbag2_py import StorageOptions, ConverterOptions, StorageFilter
from rclpy.serialization import deserialize_message

import pathlib
import argparse
import collections
from matplotlib.pyplot import subplots, show
from social_nav_util.pedestrian_smoother import PedestrianSmoother, Velocity
from social_nav_msgs.msg import Pedestrians


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('bag_path', type=pathlib.Path)
    parser.add_argument('window', type=float, default=1.0, nargs='?')
    args = parser.parse_args()

    serialization_format = 'cdr'

    bag_options = (StorageOptions(str(args.bag_path), 'sqlite3'),
                   ConverterOptions(serialization_format, serialization_format))
    reader = SequentialReader()
    reader.open(*bag_options)
    topic_types = reader.get_all_topics_and_types()
    topics = [tmeta.name for tmeta in topic_types if tmeta.type == 'social_nav_msgs/msg/Pedestrians']
    storage_filter = StorageFilter(topics=topics)
    reader.set_filter(storage_filter)

    src_data = collections.defaultdict(list)
    filtered_data = collections.defaultdict(list)
    t0 = None

    smoothers = {'std': PedestrianSmoother(args.window)}
    prev = {}

    while reader.has_next():
        (topic, rawdata, timestamp) = reader.read_next()
        msg = deserialize_message(rawdata, Pedestrians)
        timestamp /= 1e9
        if t0 is None:
            t0 = timestamp
        t = timestamp - t0
        ids = []

        for ped in msg.pedestrians:
            v = Velocity(twist2d=ped.velocity)

            # src_data[f'{ped.identifier} pose'].append((ped.pose.x, ped.pose.y))
            if ped.identifier in prev:
                v.normalize_angle(prev[ped.identifier])
            prev[ped.identifier] = v
            ids.append(ped.identifier)
            src_data[f'{ped.identifier}_mag'].append((t, v.magnitude))
            src_data[f'{ped.identifier}_angle'].append((t, v.angle))

        for key, smoother in smoothers.items():
            smoother.update(msg)

            for ped_id in ids:
                base_key = f'{key}_{ped_id}'
                v = smoother.get_velocity(ped_id)
                filtered_data[f'{base_key}_mag'].append((t, v.magnitude))
                filtered_data[f'{base_key}_angle'].append((t, v.angle))

    fig, ax = subplots(1)
    for key in src_data:
        x = [a[0] for a in src_data[key]]
        y = [a[1] for a in src_data[key]]

        ax.plot(x, y, '.', label=key)
    for key in filtered_data:
        x = [a[0] for a in filtered_data[key]]
        y = [a[1] for a in filtered_data[key]]

        ax.plot(x, y, '-', label=key)
    ax.legend()
    show()
