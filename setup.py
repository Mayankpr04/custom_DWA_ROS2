from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'custom_dwa_planner'

launch_dir = os.path.join(os.path.dirname(__file__), 'launch')
config_dir = os.path.join(os.path.dirname(__file__), 'config')

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join(launch_dir,'*.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join(config_dir,'*.yaml'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join(config_dir,'*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mayank',
    maintainer_email='mayankpratap2000@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'dwa_planner_node = custom_dwa_planner.dwa_planner_node:main'
        ],
    },
)
