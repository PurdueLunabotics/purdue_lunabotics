#!/usr/bin/env python3

from setuptools import setup

package_name = "lunabot_control"
setup(
  name=package_name,
  version='1.0.0',
  maintainer='Purdue Lunabotics',
  maintainer_email='lunabot@purdue.edu',
  description='controls the lunabot',
  license='None',
  packages=["lunabot_control"], 
  package_dir={"": "src"},
  install_requires=['setuptools'],
  zip_safe=True,
  data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
],
  entry_points={
    'console_scripts': [
        'point_to_point_node = lunabot_control.point_to_point:main',
    ],
},
)
