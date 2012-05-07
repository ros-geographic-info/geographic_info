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
Generate UUIDs for Geographic Information Systems

:Author: Jack O'Quin
"""

# prepare for Python 3 migration some day
from __future__ import print_function

# unfortunately, the Python uuid package does not work with unicode strings:
#from __future__ import unicode_literals

PKG='geodesy'
import roslib; roslib.load_manifest(PKG)
from geographic_msgs.msg import UniqueID

import uuid

def generate(url, id=None):
    """ Generate UUID for geographic data.

    :param url: URL indicating generating source
    :type  url: str
    :param id: (optional) identifier, unique within URL name space
    :type  id: int or string convertible to int

    :returns: standard Python uuid object
    :raises: :exc:`ValueError` if id not convertible to int.

  http://tools.ietf.org/html/rfc4122.html

Matching features within each name space must yield the same UUID.
The method used is RFC 4122 variant 5, computing the SHA-1 hash of a
URL encoded using the map source.  For example, Open Street Map
identifiers are encoded like this:

  generate('http://openstreetmap.org/node/', node_id)
  generate('http://openstreetmap.org/way/', way_id)
  generate('http://openstreetmap.org/relation/', rel_id)

Here the `*_id` parameters are integer representations of OSM node,
way, or relation identifiers.

For RouteSegment graph edges we use:

  generate('http://ros.org/wiki/PACKAGE_NAME/START_UUID/END_UUID')

Where PACKAGE_NAME is the generating ROS package, START_UUID names the
beginning way point, and END_UUID is the ending way point.

"""
    if id is not None:
        url += str(int(id))
    return uuid.uuid5(uuid.NAMESPACE_URL, url)


def makeUniqueID(url, id=None):
    """Return a UniqueID message for id number in name space ns.

    :param url: URL indicating generating source
    :type  url: str
    :param id: (optional) identifier, unique within URL name space
    :type  id: int or string convertible to int

    :returns: geographic_msgs/UniqueID message
    :raises: :exc:`ValueError` if id not convertible to int.

    See: generate() explanation of name space and identifier rules.

    Note: we store the string representation of the UUID in the
          message.  That uses over twice the space of a byte string,
          but makes the messages human-readable.
    """
    return UniqueID(uuid = str(generate(url, id)))
