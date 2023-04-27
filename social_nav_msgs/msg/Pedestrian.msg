string identifier

# pose and velocity are relative to the same frame
# i.e. if a pedestrian starts at x=1, y=0, facing positive y (i.e. theta=90deg)
#      and was walking forward at 1 m/s, the velocity should be x=0, y=1, theta=0
geometry_msgs/Pose2D pose
nav_2d_msgs/Twist2D velocity
