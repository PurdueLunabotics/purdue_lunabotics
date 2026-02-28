from setuptools import setup

package_name = 'test_firmware'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Purdue Lunabotics',
    maintainer_email='placeholder@email.com',
    description='Serial bridge for Teensy communication',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_bridge = test_firmware.nanopb_serial_bridge:main',
        ],
    },
)
