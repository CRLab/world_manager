## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

# requirements = [line.strip() for line in open("requirements.txt")]

d = generate_distutils_setup()

d['name'] = "world_manager"
d['description'] = "Code for interfacing with Movit planning scene and object TFs"
d['packages'] = ['world_manager']
d['package_dir'] = {'': 'src'}

setup(**d)
