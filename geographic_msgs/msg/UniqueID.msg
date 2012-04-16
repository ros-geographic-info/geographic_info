# Unique ID for a geographic feature.
#
# This is a universal unique identifier (UUID):
#
#  http://en.wikipedia.org/wiki/Universally_unique_identifier
#
# UUID generation is up to the programmer, but the intent is for
# matching features within a domain such as Open Street Map to have
# the same UUID.  One method is computing the MD5 hash of a URL
# encoded using the map source, like:
#
#  http://openstreetmap.org/node/<number>
#  http://openstreetmap.org/way/<number>
#  http://openstreetmap.org/relation/<number>

byte[128] uuid
