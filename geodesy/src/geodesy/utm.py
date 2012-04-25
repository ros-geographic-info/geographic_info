#!/usr/bin/python
# Software License Agreement (BSD License)
#
# Copyright (C) 2012, Austin Robot Technology
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
#  * Neither the name of Austin Robot Technology, Inc. nor the names
#    of its contributors may be used to endorse or promote products
#    derived from this software without specific prior written
#    permission.
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

    @author: Jack O'Quin
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

    @todo: add Universal Polar Stereographic support
    """


    def __init__(self, easting=0.0, northing=0.0,
                 altitude=float('nan'), zone=0, band=' '):
        """Construct UTMPoint object.

        @param easting: UTM easting (meters)
        @param northing: UTM northing (meters)
        @param altitude: altitude above the WGS84 ellipsoid (meters),
                         none if NaN.
        @param zone: UTM longitude zone
        @param band: MGRS latitude band letter
        """

        self.easting = easting
        self.northing = northing
        self.altitude = altitude
        self.zone = zone
        self.band = band

    def __str__(self):
        """@return: string representation of UTMPoint"""
        # uses python3-compatible str.format() method:
        return 'UTM: [{0:.3f}, {1:.3f}, {2:.3f}, {3}{4}]'.format(
            self.easting, self.northing, self.altitude, self.zone, self.band)

    def is2D(self):
        """UTM point is two-dimensional.

           @return: True if altitude is not a number (NaN)
        """
        return self.altitude != self.altitude

    def toPoint(self):
        """Generate geometry_msgs/Point from UTMPoint

           @todo: clamp message longitude to [-180..180]

           @return: corresponding GeoPoint message
        """
        if not self.valid():
            raise ValueError('invalid UTM point: ' + str(self))
        pt = Point(x = self.easting, y = self.northing)
        if not self.is2D:
            pt.z = self.altitude
        return pt

    def toMsg(self):
        """Generate GeoPoint message from UTMPoint.

           @todo: clamp message longitude to [-180..180]

           @return: corresponding GeoPoint message
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

           @return: True if this is a valid UTM point.
        """
        return (1 <= self.zone and self.zone <= 60 and self.band != ' ')

def fromLatLong(latitude, longitude, altitude=float('nan')):
    """Generate UTMPoint from latitude, longitude and (optional) altitude.
    """
    z, b = getGridZone(latitude, longitude)
    utm_proj = pyproj.Proj(proj='utm', zone=z, datum='WGS84')
    e, n = utm_proj(longitude, latitude)
    return UTMPoint(easting=e, northing=n, altitude=altitude, zone=z, band=b)

def fromMsg(msg):
    """Generate UTMPoint from GeoPoint message.
    """
    return fromLatLong(msg.latitude, msg.longitude, msg.altitude)

def getGridZone(lat, lon):
    """Get UTM zone and MGRS band for GeoPoint message.

       @param: latitude: latitude in degrees, negative is South
       @param: longitude: longitude in degrees, negative is West
       @return: (zone, band) tuple

       @todo: handle polar (UPS) zones: A, B, Y, Z.
       @todo: normalize message longitude to [-180..180]
    """
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
    else: raise ValueError('latitude out of UTM range')
    return (zone, band)

if __name__ == '__main__':

    # unit tests:

    # null constructor
    pt = UTMPoint()
    if pt.valid():
        raise ValueError('uninitialized UTMPoint should be invalid: ' + str(pt))
    if not pt.is2D():
        raise ValueError('this UTMPoint should be 2D: ' + str(pt))
    try:
        pt.toMsg()
        raise ValueError('uninitialized UTMPoint should be invalid: ' + str(pt))
    except ValueError:
        #print('uninitialized UTMPoint invalid, as expected')
        pass                            # expected result

    # UTM point in  Pickle Research Campus, University of Texas, Austin
    ll = GeoPoint(latitude = 30.385315,
                  longitude = -97.728524,
                  altitude = 209.0)
    pt = fromMsg(ll)
    if str(pt) != 'UTM: [622159.338, 3362168.303, 209.000, 14R]':
        raise ValueError('conversion failed: ' + str(pt))
    if not pt.valid():
        raise ValueError('invalid UTMPoint: ' + str(pt))
    if pt.is2D():
        raise ValueError('this UTMPoint should be 3D: ' + str(pt))
    ll_back = pt.toMsg()
    if str(ll) != str(ll_back):
        print(ll)
        print(ll_back)
        raise ValueError('GeoPoint conversion failed for: ' + str(pt))

    # same point, but without altitude
    lat = 30.385315
    lon = -97.728524
    alt = float('nan')
    pt = fromLatLong(lat, lon)
    if str(pt) != 'UTM: [622159.338, 3362168.303, nan, 14R]':
        raise ValueError('conversion failed: ' + str(pt))
    if not pt.valid():
        raise ValueError('invalid UTMPoint: ' + str(pt))
    if not pt.is2D():
        raise ValueError('this UTMPoint should be 2D: ' + str(pt))
    ll = GeoPoint(lat, lon, alt)
    ll_back = pt.toMsg()
    if str(ll) != str(ll_back):
        print(ll)
        print(ll_back)
        raise ValueError('GeoPoint conversion failed for: ' + str(pt))

    print('unit tests ran successfully')
