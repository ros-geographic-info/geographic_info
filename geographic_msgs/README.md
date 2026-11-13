Geographic information messages.

## Unique Identifiers

Map points, features and segments all have universally unique identifier
names
([UUID](http://en.wikipedia.org/wiki/Universally_unique_identifier)),
using
[uuid_msgs/msg/UniqueID](https://docs.ros.org/en/rolling/p/uuid_msgs/msg/UniqueID.html)
messages.

Each UUID is stored as an array of unsigned bytes. UUID generation is up
to the programmer, but the intent is for matching features within a
domain such as Open Street Map to yield the same UUID. The
[unique_id](https://index.ros.org/p/unique_id) package provides C++ and
Python interfaces for generating them: use the `fromURL()` method on a
URL constructed from the map source.

For example, the Open Street Map URL scheme is:

- `http:*openstreetmap.org/node/999999` \*
  `http:*openstreetmap.org/way/999999`
- `http:*openstreetmap.org/relation/999999`*

Here, `999999` is the decimal representation of the integer OSM node,
way, or relation ID without leading zeros.

Other map sources should use similar conventions.

## Cartography

The
[geographic_msgs/srv/GetGeographicMap](https://docs.ros.org/en/rolling/p/geographic_msgs/srv/GetGeographicMap.html)
service takes a map URL and optional
[geographic_msgs/msg/BoundingBox](https://docs.ros.org/en/rolling/p/geographic_msgs/msg/BoundingBox.html),
returning a
[geographic_msgs/msg/GeographicMap](https://docs.ros.org/en/rolling/p/geographic_msgs/msg/GeographicMap.html),

The map contains a
[geographic_msgs/msg/WayPoint](https://docs.ros.org/en/rolling/p/geographic_msgs/msg/WayPoint.html)
vector and a
[geographic_msgs/msg/MapFeature](https://docs.ros.org/en/rolling/p/geographic_msgs/msg/MapFeature.html)
vector. Each point or feature is identified by a
[geographic_msgs/msg/UniqueID](https://docs.ros.org/en/rolling/p/geographic_msgs/msg/UniqueID.html),
and optional
[geographic_msgs/msg/KeyValue](https://docs.ros.org/en/rolling/p/geographic_msgs/msg/KeyValue.html)
properties describing their roles.

Features represent any interesting collection of map points: hiking
trails, bicycle paths, streets, highways, tunnels, bridges, rivers,
buildings, property boundaries, etc. They contain a sequence of
component unique identifiers, which may name points or other features. A
feature may not directly or indirectly contain itself, so its
subordinate points and features must form a tree structure. But, a
feature may belong to more than one higher-level feature.

## Route Network

Not all way points in a map are drivable by any particular vehicle. Some
delimit buildings, rivers, or property boundaries. The
[geographic_msgs/msg/RouteNetwork](https://docs.ros.org/en/rolling/p/geographic_msgs/msg/RouteNetwork.html)
message represents drivable paths as a directed graph using a
[geographic_msgs/msg/WayPoint](https://docs.ros.org/en/rolling/p/geographic_msgs/msg/WayPoint.html)
vector and a
[geographic_msgs/msg/RouteSegment](https://docs.ros.org/en/rolling/p/geographic_msgs/msg/RouteSegment.html)
vector. Each segment represents an edge from one point to another.
Unless the path is one-way, another segment will point in the opposite
direction.
