# Route network segment.
#
# This is one directed edge of a RouteNetwork graph. It represents a
# known path from one way point to another.  If the path is two-way,
# there will be another RouteSegment with "start" and "end" reversed.

UniqueID   id           # Unique identifier for this segment

UniqueID   start        # beginning way point of segment
UniqueID   end          # ending way point of segment

KeyValue[] props        # segment properties
