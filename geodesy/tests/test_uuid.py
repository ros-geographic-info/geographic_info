#!/usr/bin/env python

PKG='geodesy'
import roslib; roslib.load_manifest(PKG)

import sys
import unittest

from geodesy.gen_uuid import *
from geographic_msgs.msg import UniqueID

class TestPythonUUID(unittest.TestCase):
    """Unit tests for Python UUID generation.
    """

    # raw UUID generation tests
    def test_int_with_leading_zeros(self):
        x = generate('http://openstreetmap.org/node/', 1)
        y = generate('http://openstreetmap.org/node/', 0001)
        self.assertEqual(x, y)
        self.assertEqual(str(x), 'ef362ac8-9659-5481-b954-88e9b741c8f9')

    def test_int_in_a_string(self):
        x = generate('http://openstreetmap.org/node/', 1)
        y = generate('http://openstreetmap.org/node/', '0001')
        self.assertEqual(x, y)
        self.assertEqual(str(y), 'ef362ac8-9659-5481-b954-88e9b741c8f9')
        
    def test_same_id_different_namespace(self):
        x = generate('http://openstreetmap.org/node/', 1)
        y = generate('http://openstreetmap.org/way/', 1)
        self.assertNotEqual(x, y)
        self.assertEqual(str(y), 'b3180681-b125-5e41-bd04-3c8b046175b4')
        
    def test_actual_osm_node_id(self):
        x = generate('http://openstreetmap.org/node/', 1)
        y = generate('http://openstreetmap.org/node/', 152370223)
        self.assertNotEqual(x, y)
        self.assertEqual(str(y), '8e0b7d8a-c433-5c42-be2e-fbd97ddff9ac')

    def test_invalid_id_value(self):
        self.assertRaises(ValueError, generate,
                          'http://openstreetmap.org/way/', 'xxx')

    # UniqueID message generation tests
    def test_msg_creation(self):
        msg = makeUniqueID('http://openstreetmap.org/node/', 152370223)
        self.assertEqual(msg.uuid, '8e0b7d8a-c433-5c42-be2e-fbd97ddff9ac')
        
    def test_msg_same_id_different_namespace(self):
        x = makeUniqueID('http://openstreetmap.org/node/', 1)
        y = makeUniqueID('http://openstreetmap.org/way/', 1)
        self.assertNotEqual(x, y)
        self.assertEqual(y.uuid, 'b3180681-b125-5e41-bd04-3c8b046175b4')

    def test_msg_invalid_value(self):
        self.assertRaises(ValueError, makeUniqueID,
                          'http://openstreetmap.org/way/', 'xxx')

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_uuid_py', TestPythonUUID) 
