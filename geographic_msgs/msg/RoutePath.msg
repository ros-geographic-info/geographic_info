# Path through a route network.
#
# A path is a sequence of RouteSegment edges.  This information is
# extracted from a RouteNetwork graph.  A RoutePath lists the route
# segments needed to reach some chosen goal.

Header          header

UniqueID        network  # route network containing this path
UniqueID[]      segments # sequence of RouteSegment IDs
KeyValue[]      tags     # Key/value properties
