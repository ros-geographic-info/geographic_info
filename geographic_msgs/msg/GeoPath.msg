std_msgs/Header header
geographic_msgs/GeoPoseStamped[] poses
=======
# Path through a route network.
#
# A path is a sequence of GeoPoints.

Header header

geographic_msgs/GeoPoint[] points  # a list of points on the route
uuid_msgs/UniqueID network         # id of the used route network
uuid_msgs/UniqueID start_seg       # id of the nearest segment to starting way point
uuid_msgs/UniqueID goal_seg        # id of the nearest segment to the goal way point
float64 distance                   # the calculated distance
