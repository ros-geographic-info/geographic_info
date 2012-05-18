geodesy
-------

.. currentmodule:: geodesy.gen_uuid


gen_uuid
--------

.. function:: generate(url, id=None)

    :param url: URL indicating generating source
    :param id: (optional) identifier, unique within URL name space
    :type  id: int or string convertible to int

    :returns: standard Python uuid object
    :raises: :exc:`ValueError` if id not convertible to int.

  http://tools.ietf.org/html/rfc4122.html

Matching features within each name space must yield the same UUID.
The method used is RFC 4122 variant 5, computing the SHA-1 hash of a
URL encoded using the map source.  For example, Open Street Map
identifiers are encoded like this::

  generate('http://openstreetmap.org/node/', node_id)
  generate('http://openstreetmap.org/way/', way_id)
  generate('http://openstreetmap.org/relation/', rel_id)

Here the `*_id` parameters are integer representations of OSM node,
way, or relation identifiers.

For RouteSegment graph edges we use::

  generate('http://ros.org/wiki/PACKAGE_NAME/START_UUID/END_UUID')

Where PACKAGE_NAME is the generating ROS package, START_UUID names the
beginning way point, and END_UUID is the ending way point.
