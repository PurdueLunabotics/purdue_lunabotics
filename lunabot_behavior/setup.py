from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'lunabot_behavior'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='lunabot@purdue.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "zones = lunabot_behavior.zones:main",
            "find_linkup = lunabot_behavior.states.find_linkup:main",
            "main = lunabot_behavior.main_states:main",
            "mini = lunabot_behavior.mini_states:main",
            "linkup_main = lunabot_behavior.linkup_main_states:main",
            "linkup_mini = lunabot_behavior.linkup_mini_states:main",
            "dep = lunabot_behavior.dep_only_states:main",
            "ex = lunabot_behavior.ex_only_states:main",
            "ex_align = lunabot_behavior.ex_align_states:main",
            "single = lunabot_behavior.single_states:main",
        ],
    },
)
