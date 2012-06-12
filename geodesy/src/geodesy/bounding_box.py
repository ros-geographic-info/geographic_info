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
.. module:: bounding_box

Bounding box functions for geographic maps.

.. _`geographic_msgs/BoundingBox`: http://ros.org/doc/api/geographic_msgs/html/msg/BoundingBox.html

"""

PKG = 'geodesy'
import roslib; roslib.load_manifest(PKG)
import rospy

from geographic_msgs.msg import BoundingBox

def getLatLong(bbox):
    """
    Get the tuple of minimum and maximum latitudes and longitudes.

    :param bbox: `geographic_msgs/BoundingBox`_.
    :returns: (min_lat, min_lon, max_lat, max_lon)
    """
    return (bbox.min_latitude, bbox.min_longitude,
            bbox.max_latitude, bbox.max_longitude)

def isEmpty(bbox):
    """
    Empty bounding box predicate.

    :param bbox: `geographic_msgs/BoundingBox`_.
    :returns: True if *bbox* is empty.
    """
    return (bbox.min_latitude == 0.0 and bbox.min_longitude == 0.0 and 
            bbox.max_latitude == 0.0 and bbox.max_longitude == 0.0)

def makeBounds2D(min_lat, min_lon, max_lat, max_lon):
    """
    Create a 2D bounding box (ignoring altitudes).

    :param min_lat: Minimum latitude.
    :param min_lon: Minimum longitude.
    :param max_lat: Maximum latitude.
    :param max_lon: Maximum longitude.
    :returns: `geographic_msgs/BoundingBox`_ object.
    """
    return BoundingBox(min_lat, min_lon, max_lat, max_lon)

def makeEmpty():
    """
    Create an empty bounding box.

    :returns: `geographic_msgs/BoundingBox`_ object.
    """
    return BoundingBox()
