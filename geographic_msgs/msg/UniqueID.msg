# A universally unique identifier (UUID) for a geographic feature.
#
#  http://en.wikipedia.org/wiki/Universally_unique_identifier
#  http://tools.ietf.org/html/rfc4122.html
#
# For simplicity and human readability, the UUID is represented as a
# string of hex digits and dashes.
#
# UUID generation is up to the programmer, but the intent is for
# matching features within a domain such as Open Street Map to yield
# the same UUID.  The recommended method is RFC 4122 variant 5,
# computing the SHA-1 hash of a URL encoded using the map source.  For
# example:
#
#  http://openstreetmap.org/node/NUMBER
#  http://openstreetmap.org/way/NUMBER
#  http://openstreetmap.org/relation/NUMBER
#
# Here NUMBER is the decimal representation of the OSM node, way, or
# relation ID without leading zeros.
#
# Other map sources should use similar conventions.

string uuid     # format: 01234567-89ab-cdef-0123-456789abcdef
