# Two-dimensional geographic map bounding box. 

# Latitudes are in degrees [-90 .. +90]. Positive is north of equator;
# negative is south.

# Longitudes are in degrees [-180 .. +180]. Positive is east of prime
# meridian; negative is west. At the poles, latitude is -90 or +90,
# and longitude is irrelevant, but must be in range.

# If all bounds are zeros, the bounding box includes the whole world.

float64 min_latitude
float64 min_longitude
float64 max_latitude
float64 max_longitude
