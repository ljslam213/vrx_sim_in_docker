from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'wamv_ctl'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ljin',
    maintainer_email='jin.li@ehang.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'inverse_kinematics = wamv_ctl.inverse_kinematics:main',
            'station_keeping = wamv_ctl.station_keeping:main',
            'wayfinding = wamv_ctl.wayfinding:main',
            'path_follow_adpLOS = wamv_ctl.path_follow_adpLOS:main',
            'eight_path = wamv_ctl.path_generator.eight_path:main',
            'dubins_path = wamv_ctl.path_generator.dubins_path:main',
        ],
    },
)
