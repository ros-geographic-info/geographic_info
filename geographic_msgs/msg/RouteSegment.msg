# Route network segment.
#
# This is one directed edge of a RouteNetwork graph. It represents a
# known path from one way point to another.  If the path is two-way,
# there will be another RouteSegment with "start" and "end" reversed.

UniqueID   id           # Unique identifier for this segment

UniqueID   start        # beginning way point of segment
UniqueID   end          # ending way point of segment

# The speed limit is useful for route planning, when a minimum time
# rather than minimum distance is desired.  When unknown, its value is
# zero.
float32    speed_limit  # speed limit for this segment [meters/second]
