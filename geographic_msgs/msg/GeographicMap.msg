# Geographic map for a specified region.

Header header           # stamp specifies time
                        # frame_id is normally /map

# The latitudes and longitudes of these two points delimit a bounding
# box containing the map contents. Their altitudes are ignored.
GeoPoint[2] bounds

WaySequence[] elements  # Roads and other map components
