#!/usr/bin/env python

from catkin_pkg.python_setup import generate_distutils_setup

try:
    from distutils.core import setup
except ImportError:
    try:
      from setuptools import setup
    except ImportError:
      assert False, 'Must have distutils or setuptools installed!'

d = generate_distutils_setup(
    packages=['geodesy'],
    package_dir={'': 'src'},
    )

setup(**d)
