#!/usr/bin/env python

PKG='geodesy'
import roslib; roslib.load_manifest(PKG)

import unittest

from geodesy.bounding_box import *     # module being tested

class TestPythonBoundingBox(unittest.TestCase):
    """Unit tests for Python bounding box functions. """

    def test_empty_bounding_box(self):
        bb = makeEmpty()
        self.assertTrue(isEmpty(bb))
        min_lat, min_lon, max_lat, max_lon = getLatLong(bb)
        self.assertEqual(min_lat, 0.0)
        self.assertEqual(min_lon, 0.0)
        self.assertEqual(max_lat, 0.0)
        self.assertEqual(max_lon, 0.0)

    def test_2d_bounding_box(self):
        min_lat = 30.3787400
        min_lon = -97.7344500
        max_lat = 30.3947700
        max_lon = -97.7230800
        bb = makeBounds2D(min_lat, min_lon, max_lat, max_lon)
        self.assertFalse(isEmpty(bb))
        min_lat2, min_lon2, max_lat2, max_lon2 = getLatLong(bb)
        self.assertEqual(min_lat, min_lat2)
        self.assertEqual(min_lon, min_lon2)
        self.assertEqual(max_lat, max_lat2)
        self.assertEqual(max_lon, max_lon2)

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_uuid_py', TestPythonBoundingBox) 
