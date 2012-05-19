geodesy
-------


gen_uuid
--------

.. currentmodule:: geodesy.gen_uuid

.. function:: generate(url, [id=None])

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


.. function:: makeUniqueID(url, [id=None])

    :param url: URL indicating generating source
    :param id: (optional) identifier, unique within URL name space
    :type  id: int or string convertible to int

    :returns: geographic_msgs/UniqueID message
    :raises: :exc:`ValueError` if id not convertible to int.

    seealso:: generate() explanation of name space and identifier rules.

    note:: we store the string representation of the UUID in the
           message.  That uses over twice the space of a 16-byte
           array, but makes the messages human-readable.


.. currentmodule:: geodesy.props


props
-----

.. currentmodule:: geodesy.props

.. function:: get(msg, key):

    Get property value.

    :param msg: Message containing properties.
    :param key: Property key to match.

    :returns: Corresponding value, if defined; None otherwise.
              Beware: the value may be '', which evaluates False as a
              predicate, use ``is not None`` to test for presence.

.. function:: match(msg, key_set):

    Match message properties.

    :param msg:     Message containing properties.
    :param key_set: Set of property keys to match.
    :returns: (key, value) of first property matched; None otherwise.
    :raises: :exc:`ValueError` if key_set is not a set

.. function:: put(msg, key, [val='']):

    Add KeyValue to message properties.

    :param msg:   Message to update.
    :param key:   Property key name.
    :param value: Corresponding value string (default '').


.. currentmodule:: geodesy.utm

utm
---

.. currentmodule:: geodesy.utm

.. class:: UTMPoint

    Universal Transverse Mercator point class.

    :todo: add Universal Polar Stereographic support

   .. member:: __init__(self, easting=float('nan'), northing=float('nan'),
                       altitude=float('nan'), zone=0, band=' '):

        Constructor for UTMPoint object.

        :param easting: UTM easting (meters)
        :param northing: UTM northing (meters)
        :param altitude: altitude above the WGS84 ellipsoid (meters),
                         none if NaN.
        :param zone: UTM longitude zone
        :param band: MGRS latitude band letter

   .. member:: __str__(self)

        Overloaded str() operator.
        
        :returns: string representation of UTMPoint

   .. member:: gridZone(self)

        :returns: MGRS zone and band tuple.

   .. member:: is2D(self)

        UTM point is two-dimensional.

        :returns: True if altitude is not a number (NaN).

   .. member:: toPoint(self):

        Generate geometry_msgs/Point from UTMPoint

        :returns: corresponding geometry_msgs/Point

   .. member:: toPoint(self):

        Generate geometry_msgs/Point from UTMPoint

        :returns: corresponding geometry_msgs/Point

   .. member:: toMsg(self):

        Generate GeoPoint message from UTMPoint.

        :returns: corresponding GeoPoint message

   .. member:: valid(self):

        UTM point is valid.

        Easting and northing will be NaN if a point is not valid.

        :returns: True if this is a valid UTM point.

.. function:: fromLatLong(latitude, longitude, altitude=float('nan')):

    Generate UTMPoint from latitude, longitude and (optional) altitude.

    Latitude and longitude are expressed in degrees, relative to the
    WGS84 ellipsoid.

    :param latitude: [degrees], negative is South.
    :param longitude: [degrees], negative is West.
    :param altitude: [meters], negative is below the ellipsoid.

    :returns: UTMPoint object.

.. function:: fromMsg(msg):

    Generate UTMPoint from GeoPoint message.

    :param msg: GeoPoint message.
    :returns: UTMPoint object.

.. function:: getGridZone(lat, lon):

    Get UTM zone and MGRS band for GeoPoint message.

    :param lat: latitude in degrees, negative is South
    :param lon: longitude in degrees, negative is West
    :returns: (zone, band) tuple
    :raises: :exc:`ValueError` if lon not in [-180..180] or if lat
             has no corresponding band letter.

    :todo: handle polar (UPS) zones: A, B, Y, Z.

wu_point
--------

.. automodule:: geodesy.wu_point
   :members:
