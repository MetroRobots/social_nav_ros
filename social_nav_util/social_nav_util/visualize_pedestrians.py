import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from social_nav_msgs.msg import Pedestrians
from math import sin, cos


def makePoint(pose2d, angle=0.0, radius=0.5):
    p = Point()
    p.x = pose2d.x + radius * cos(pose2d.theta + angle)
    p.y = pose2d.y + radius * sin(pose2d.theta + angle)
    return p


TAIL_ANGLE = 2.8274  # pi * 0.9


class VisualizePedestrians(Node):

    def __init__(self):
        super().__init__('visualize_pedestrians')
        self.pub = self.create_publisher(MarkerArray, '/markers', 1)
        self.subscription = self.create_subscription(Pedestrians, '/pedestrians', self.ped_cb, 1)

    def ped_cb(self, msg):
        ma = MarkerArray()
        body_marker = Marker()
        body_marker.header = msg.header
        body_marker.ns = 'bodies'
        body_marker.type = Marker.TRIANGLE_LIST
        body_marker.scale.x = 1.0
        body_marker.scale.y = 1.0
        body_marker.scale.z = 1.0
        body_marker.color.a = 1.0
        body_marker.color.r = 1.0
        ma.markers.append(body_marker)

        for i, ped in enumerate(msg.pedestrians):
            marker_label = Marker()
            marker_label.header = msg.header
            marker_label.ns = 'label'
            marker_label.id = i
            marker_label.type = Marker.TEXT_VIEW_FACING
            marker_label.pose.position.x = ped.pose.x
            marker_label.pose.position.y = ped.pose.y

            marker_label.scale.z = 0.25
            marker_label.color.a = 1.0
            marker_label.color.r = 1.0
            marker_label.color.g = 1.0
            marker_label.text = ped.identifier

            ma.markers.append(marker_label)

            marker_vel = Marker()
            marker_vel.header = msg.header
            marker_vel.ns = 'velocity'
            marker_vel.id = i
            marker_vel.type = Marker.ARROW
            marker_vel.points.append(marker_label.pose.position)
            ep = Point()
            ep.x = float(ped.pose.x + ped.velocity.x)
            ep.y = float(ped.pose.y + ped.velocity.y)
            marker_vel.points.append(ep)

            marker_vel.scale.x = 0.1
            marker_vel.scale.y = 0.2
            marker_vel.color.a = 1.0
            marker_vel.color.g = 1.0
            ma.markers.append(marker_vel)

            body_marker.points.append(makePoint(ped.pose))
            body_marker.points.append(makePoint(ped.pose, TAIL_ANGLE))
            body_marker.points.append(makePoint(ped.pose, -TAIL_ANGLE))

        self.pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(VisualizePedestrians())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
