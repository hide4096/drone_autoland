from setuptools import setup
import os
from glob import glob

package_name = 'drone_autoland'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hide4096',
    maintainer_email='asouhide2002@gmail.com',
    description='Guide a drone',
    license='soiya',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'read_marker = drone_autoland.read_marker:main',
          'aruco_detect = drone_autoland.aruco_detect:main'
        ],
    },
)
