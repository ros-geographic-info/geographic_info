# Software License Agreement (BSD License)
#
# Copyright (C) 2012, Jack O'Quin
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the author nor of other contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
.. module:: wu_point

Convenience classes for manipulating way points and their associated
UTM coordinates.

"""

PKG = 'geodesy'
import roslib; roslib.load_manifest(PKG)
import rospy

import math
import geodesy.utm

from geographic_msgs.msg import WayPoint
from geometry_msgs.msg import Point

class WuPoint():
    """
    :class:`WuPoint` represents a map WayPoint with associated UTM
    information.
    """

    def __init__(self, waypt, utm=None):
        """Constructor.

        Collects relevant information from the way point message, and
        creates the corresponding geodesy.utm.UTMPoint.

        :param waypt: geographic_msgs/WayPoint message
        """
        self.way_pt = waypt

        if utm:
            self.utm = utm
        else:
            # convert latitude and longitude to UTM (ignoring altitude)
            self.utm = geodesy.utm.fromMsg(waypt.position)

    def __str__(self):
        """Overloaded str() operator.
        
        :returns: string representation of :class:`WuPoint`
        """
        # uses python3-compatible str.format() method:
        return str(self.way_pt) + '\n' + str(self.utm)

    def is2D(self):
        """Is this way point 2D?

        For 2D points, the altitude is represented as a float NaN.

        :returns: True if no altitude provided.
        """
        geo_point = self.way_pt.position
        return geo_point.altitude != geo_point.altitude

    def position(self):
        """Get way point position.

        :returns: Corresponding GeoPoint object.
        """
        return self.way_pt.position

    def toPoint(self):
        """Generate geometry_msgs/Point from :class:`WuPoint`

           :returns: corresponding geometry_msgs/Point
        """
        return self.utm.toPoint()

    def toPointXY(self):
        """Convert :class:`WuPoint` to flattened geometry_msgs/Point.

           :returns: geometry_msgs/Point with X and Y coordinates, Z is 0.
        """
        return Point(x = self.utm.easting, y = self.utm.northing)

    def toWayPoint(self):
        """Convert :class:`WuPoint` to geographic_msgs/WayPoint

           :returns: corresponding geographic_msgs/WayPoint
        """
        return self.way_pt

    def uuid(self):
        """Get way point UniqueID.

        :returns: Corresponding UniqueID message.
        """
        return self.way_pt.id.uuid

class WuPointSet():
    """
    :class:`WuPointSet` is a container for the way points in a
            geographic_msgs GeographicMap or RouteNetwork message.
    """

    def __init__(self, points):
        """Constructor.

        Collects relevant way point information from the geographic
        map message, and provides convenient access to the data.

        :param points: array of geographic_msgs/WayPoint messages
        """
        self.points = points

        # Initialize way point information.
        self.way_point_ids = {}         # points symbol table
        self.n_points = len(self.points)
        for wid in xrange(self.n_points):
            self.way_point_ids[self.points[wid].id.uuid] = wid

        # Create empty list of UTM points, corresponding to map points.
        # They will be evaluated lazily, when first needed.
        self.utm_points = [None for wid in xrange(self.n_points)]

    def __contains__(self, item):
        """ Points set membership. """
        return item in self.way_point_ids

    def __getitem__(self, key):
        """ Points accessor.

        :param key: UUID of desired point.
        :returns: Named :class:`WuPoint`.
        :raises: :exc:`KeyError` if no such point
        """
        index = self.way_point_ids[key]
        return self._get_point_with_utm(index)

    def __iter__(self):
        """ Points iterator. """
        self.iter_index = 0
        return self

    def __len__(self):
        """Points vector length."""
        return self.n_points

    def _get_point_with_utm(self, index):
        """ Get way point with UTM coordinates.

        :param index: Index of point in self.
        :returns: Corresponding :class:`WuPoint` object.
        """
        way_pt = self.points[index]
        utm_pt = self.utm_points[index]
        if utm_pt is not None:
            utm_pt = geodesy.utm.fromMsg(way_pt.position)
            self.utm_points[index] = utm_pt
        return WuPoint(way_pt, utm=utm_pt)

    def distance2D(self, idx1, idx2):
        """ Compute 2D distance between points.

        :param idx1: Index of first point.
        :param idx2: Index of second point.

        :returns: Distance in meters within the UTM XY
                  plane. Altitudes are ignored.
        """
        p1 = self._get_point_with_utm(idx1)
        p2 = self._get_point_with_utm(idx2)
        dx = p2.utm.easting - p1.utm.easting
        dy = p2.utm.northing - p1.utm.northing
        return math.sqrt(dx*dx + dy*dy)

    def get(self, key, default=None):
        """ Get point, if defined.

        :param key: UUID of desired point.
        :param default: value to return if no such point.
        :returns: Named :class:`WuPoint`, if successful; otherwise default.
        """
        index = self.way_point_ids.get(key)
        if index is not None:
            return self._get_point_with_utm(index)
        else:
            return default

    def index(self, key, default=None):
        """ Get index of point, if defined.

        :param key: UUID of desired point.
        :param default: value to return if no such point.
        :returns: Index of point, if successful; otherwise default.
                  Beware: the index may be 0, which evaluates False as
                  a predicate, use ``is not None`` to test for
                  presence.
        """
        return self.way_point_ids.get(key, default)

    def next(self):
        """ Next point.

        :returns: Next :class:`WuPoint`.
        :raises: :exc:`StopIteration` when finished.
        """
        i = self.iter_index
        if i >= self.n_points:
            raise StopIteration
        self.iter_index = i + 1
        return self._get_point_with_utm(i)
