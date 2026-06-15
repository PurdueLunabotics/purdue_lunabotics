from setuptools import find_packages, setup

package_name = 'lunabot_onboarding'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Purdue Lunabotics',
    maintainer_email='lunabot@purdue.edu',
    description='Onboarding',
    license='None',
    extras_require={ },
    entry_points={
        'console_scripts': [
        ],
    },
)
