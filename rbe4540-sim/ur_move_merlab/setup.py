from glob import glob

from setuptools import find_packages, setup

package_name = 'ur_move_merlab'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bcalli',
    maintainer_email='bcalli@wpi.edu',
    description='Simple simulated UR pick-and-place demo',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'ur_move_simple_interface = ur_move_merlab.ur_move_simple_interface:main',
            'pick_place_demo = ur_move_merlab.pick_place_demo:main',
            'simple_run = ur_move_merlab.simple_run:main',
        ],
    },
)
