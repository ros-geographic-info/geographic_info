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
.. module:: gen_uuid

Generate UUIDs for Geographic Information messages.

.. deprecated:: 0.2.2

   Use the :py:mod:`unique_id` package, instead.

Map points, features and segments all have universally unique
identifier names (UUID_), using geographic_msgs/UniqueID messages.

Matching features within each name space must yield the same UUID.
The method used is `RFC 4122`_ variant 5, computing the SHA-1 hash of
a URL encoded using the map source.  

For example, Open Street Map identifiers are encoded like this::

  generate('http://openstreetmap.org/node/', node_id)
  generate('http://openstreetmap.org/way/', way_id)
  generate('http://openstreetmap.org/relation/', rel_id)

Here the `*_id` parameters are integer representations of OSM node,
way, or relation identifiers.

For RouteSegment graph edges we use::

  generate('http://ros.org/wiki/PACKAGE_NAME/START_UUID/END_UUID')

Where PACKAGE_NAME is the generating ROS package, START_UUID names the
beginning way point, and END_UUID is the ending way point.

.. _`geographic_msgs/UniqueID`: http://ros.org/doc/api/geographic_msgs/html/msg/UniqueID.html
.. _`RFC 4122`: http://tools.ietf.org/html/rfc4122.html
.. _UUID: http://en.wikipedia.org/wiki/Uuid

"""

from uuid_msgs.msg import UniqueID
import unique_id
import uuid

def generate(url, id=None):
    """ Generate UUID_ for geographic data.

    :param url: URL indicating generating source
    :param id: (optional) identifier, unique within URL name space
    :type  id: int or string convertible to int

    :returns: standard Python uuid object
    :raises: :exc:`ValueError` if *id* not convertible to int.
"""
    if id is not None:
        url += str(int(id))
    return uuid.uuid5(uuid.NAMESPACE_URL, url)


def makeUniqueID(url, id=None):
    """Create a UniqueID message for *id* number in name space *ns*.

    :param url: URL indicating generating source
    :param id: (optional) identifier, unique within URL name space
    :type  id: int or string convertible to int
    :returns: `geographic_msgs/UniqueID`_ message
    :raises: :exc:`ValueError` if *id* not convertible to int.
    """
    return unique_id.toMsg(generate(url, id))
