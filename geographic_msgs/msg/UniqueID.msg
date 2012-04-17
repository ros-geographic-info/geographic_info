# A universally unique identifier (UUID) for a geographic feature.
#
#  http://en.wikipedia.org/wiki/Universally_unique_identifier
#  http://tools.ietf.org/html/rfc4122.html
#
# UUID generation is up to the programmer, but the intent is for
# matching features within a domain such as Open Street Map to yield
# the same UUID.  The recommended method is RFC 4122 variant 5,
# computing the SHA-1 hash of a URL encoded using the map source.  For
# example:
#
#  http://openstreetmap.org/node/<number>
#  http://openstreetmap.org/way/<number>
#  http://openstreetmap.org/relation/<number>
#
# Here <number> is the decimal representation of the OSM node, way, or
# relation ID without leading zeros.
#
# Other map sources should use similar conventions.

byte[128] uuid
