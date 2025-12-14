from glob import glob
from setuptools import setup, find_packages
import os
package_name = 'path_follower'
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=('test',)),
    data_files=[
        ('share/ament_index/resource_index/packages',
            [os.path.join('resource', package_name)]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Simple path follower package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'trans_vel = path_follower.trans_vel:main',
            'position_control = path_follower.position_control:main',
            'virtual_path = path_follower.virtual_path_publisher:main',
        ],
    },
)
