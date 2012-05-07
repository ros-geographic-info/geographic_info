#!/usr/bin/env python

PKG='geodesy'
import roslib; roslib.load_manifest(PKG)

import sys
import unittest

from geodesy.props import *     # module being tested

from geographic_msgs.msg import KeyValue
from geographic_msgs.msg import MapFeature
from geographic_msgs.msg import WayPoint

class TestPythonProps(unittest.TestCase):
    """Unit tests for Python KeyValue property handling.
    """

    def test_empty_feature_match(self):
        f = MapFeature()
        self.assertIsNone(match(f, {'no', 'such', 'property'}))

    def test_empty_property_set(self):
        f = MapFeature()
        f.tags.append(KeyValue(key = 'valid', value = 'any'))
        self.assertIsNone(match(f, set()))

    def test_feature_match(self):
        f = MapFeature()
        f.tags.append(KeyValue(key = 'different', value = 'none'))
        f.tags.append(KeyValue(key = 'valid', value = 'any'))
        prop = match(f, {'a', 'valid', 'property'})
        self.assertIsNotNone(prop)
        self.assertEqual(prop, ('valid', 'any'))
        k, v = prop
        self.assertEqual(k, 'valid')
        self.assertEqual(v, 'any')

    def test_empty_waypoint_match(self):
        p = WayPoint()
        self.assertIsNone(match(p, {'nothing', 'defined'}))

    def test_waypoint_match(self):
        p = WayPoint()
        p.tags.append(KeyValue(key = 'another', value = 'anything'))
        p.tags.append(KeyValue(key = 'name', value = 'myself'))
        prop = match(p, {'name'})
        self.assertIsNotNone(prop)
        k, v = prop
        self.assertEqual(k, 'name')
        self.assertEqual(v, 'myself')

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_uuid_py', TestPythonProps) 
