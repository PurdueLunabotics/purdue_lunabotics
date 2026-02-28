from setuptools import find_packages, setup

package_name = 'lunabot_behavior'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            "find_linkup = lunabot_behavior.find_linkup:main",
            "main = lunabot_behavior.main_states:main",
            "mini = lunabot_behavior.mini_states:main",
            "single = lunabot_behavior.single_states:main",
        ],
    },
)
