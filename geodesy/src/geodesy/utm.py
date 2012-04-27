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
#
# Revision $Id$

"""Universal Transverse Mercator coordinates

    For outdoor robotics applications, Euclidean projections like UTM
    are easier to work with than latitude and longitude.  This system
    is slightly more general than strict UTM.  It is based on the
    Military Grid Reference System (MGRS), which can be extended to
    cover the poles, allowing well-defined transformations for every
    latitude and longitude.

    This implementation uses the pyproj wrapper for the proj4
    geographic coordinate projection library.

    :Author: Jack O'Quin
"""

# prepare for Python 3 migration some day
from __future__ import print_function

import math
import pyproj

PKG_NAME = 'geodesy'
import roslib; roslib.load_manifest(PKG_NAME)
import rospy

from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg   import Point

class UTMPoint:
    """Universal Transverse Mercator point class.

    :todo: add Universal Polar Stereographic support
    """


    def __init__(self, easting=float('nan'), northing=float('nan'),
                 altitude=float('nan'), zone=0, band=' '):
        """Construct UTMPoint object.

        :param easting: UTM easting (meters)
        :param northing: UTM northing (meters)
        :param altitude: altitude above the WGS84 ellipsoid (meters),
                         none if NaN.
        :param zone: UTM longitude zone
        :param band: MGRS latitude band letter
        """

        self.easting = easting
        self.northing = northing
        self.altitude = altitude
        self.zone = zone
        self.band = band

    def __str__(self):
        """Overloaded str() operator.
        
        :returns: string representation of UTMPoint
        """
        # uses python3-compatible str.format() method:
        return 'UTM: [{0:.3f}, {1:.3f}, {2:.3f}, {3}{4}]'.format(
            self.easting, self.northing, self.altitude, self.zone, self.band)

    def gridZone(self):
        """Return MGRS zone and band tuple.
        """
        return (self.zone, self.band)

    def is2D(self):
        """UTM point is two-dimensional.

           :returns: True if altitude is not a number (NaN)
        """
        return self.altitude != self.altitude

    def toPoint(self):
        """Generate geometry_msgs/Point from UTMPoint

           :todo: clamp message longitude to [-180..180]

           :returns: corresponding geometry_msgs/Point
        """
        if not self.valid():
            raise ValueError('invalid UTM point: ' + str(self))
        pt = Point(x = self.easting, y = self.northing)
        if not self.is2D():
            pt.z = self.altitude
        return pt

    def toMsg(self):
        """Generate GeoPoint message from UTMPoint.

           :todo: clamp message longitude to [-180..180]

           :returns: corresponding GeoPoint message
        """
        if not self.valid():
            raise ValueError('invalid UTM point: ' + str(self))
        utm_proj = pyproj.Proj(proj='utm', zone=self.zone, datum='WGS84')
        msg = GeoPoint(altitude=self.altitude)
        msg.longitude, msg.latitude = utm_proj(self.easting, self.northing,
                                               inverse=True)
        return msg

    def valid(self):
        """UTM point is valid.

        Easting and northing will be NaN if a point is not valid.

        :returns: True if this is a valid UTM point.
        """
        return (self.easting == self.easting
                and self.northing == self.northing
                and self.band != ' ')

def fromLatLong(latitude, longitude, altitude=float('nan')):
    """Generate UTMPoint from latitude, longitude and (optional) altitude.

    Latitude and longitude are expressed in degrees, relative to the
    WGS84 ellipsoid.

    :param latitude: [degrees], negative is South.
    :param longitude: [degrees], negative is West.
    :param altitude: [meters], negative is below the ellipsoid.

    :returns: UTMPoint object.
    """
    z, b = getGridZone(latitude, longitude)
    utm_proj = pyproj.Proj(proj='utm', zone=z, datum='WGS84')
    e, n = utm_proj(longitude, latitude)
    return UTMPoint(easting=e, northing=n, altitude=altitude, zone=z, band=b)

def fromMsg(msg):
    """Generate UTMPoint from GeoPoint message.

    :param msg: GeoPoint message.
    :returns: UTMPoint object.
    """
    return fromLatLong(msg.latitude, msg.longitude, msg.altitude)

def getGridZone(lat, lon):
    """Get UTM zone and MGRS band for GeoPoint message.

       :param lat: latitude in degrees, negative is South
       :param lon: longitude in degrees, negative is West
       :returns: (zone, band) tuple
       :raises: :exc:`ValueError` if lon not in [-180..180] or if lat
                has no corresponding band letter.

       :todo: handle polar (UPS) zones: A, B, Y, Z.
    """
    if -180.0 > lon or lon > 180.0:
        raise ValueError('invalid longitude: ' + str(lon))
    zone = int((lon + 180.0)//6.0) + 1
    band = ' '
    if    84 >= lat and lat >= 72: band = 'X'
    elif  72 > lat and lat >= 64:  band = 'W'
    elif  64 > lat and lat >= 56:  band = 'V'
    elif  56 > lat and lat >= 48:  band = 'U'
    elif  48 > lat and lat >= 40:  band = 'T'
    elif  40 > lat and lat >= 32:  band = 'S'
    elif  32 > lat and lat >= 24:  band = 'R'
    elif  24 > lat and lat >= 16:  band = 'Q'
    elif  16 > lat and lat >= 8:   band = 'P'
    elif   8 > lat and lat >= 0:   band = 'N'
    elif   0 > lat and lat >= -8:  band = 'M'
    elif  -8 > lat and lat >= -16: band = 'L'
    elif -16 > lat and lat >= -24: band = 'K'
    elif -24 > lat and lat >= -32: band = 'J'
    elif -32 > lat and lat >= -40: band = 'H'
    elif -40 > lat and lat >= -48: band = 'G'
    elif -48 > lat and lat >= -56: band = 'F'
    elif -56 > lat and lat >= -64: band = 'E'
    elif -64 > lat and lat >= -72: band = 'D'
    elif -72 > lat and lat >= -80: band = 'C'
    else: raise ValueError('latitude out of UTM range: ' + str(lat))
    return (zone, band)
