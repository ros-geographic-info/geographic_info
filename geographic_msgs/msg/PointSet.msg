# Set of way-points for a geographic map.
#
# A set of map WayPoint IDs, used for streets, highways, hiking
# trails, the outlines of buildings and parking lots, etc.  Point sets
# may also contain other point sets as members.  Way-points for roads
# and outlines should be listed in seqential order.

UniqueID   id           # Unique set identifier
UniqueID[] members      # Members of the set
KeyValue[] tags         # Key/value tags for this set
