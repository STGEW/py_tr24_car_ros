from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'py_tr24_car_ros'

setup(
    name=package_name,
    version='0.0.4',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name), glob('urdf/*')),
    ],
    install_requires=[
        'setuptools',
        'pyserial'
    ],
    zip_safe=True,
    maintainer='afomin',
    maintainer_email='alexanderfomin.develop@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'car_logger = py_tr24_car_ros.car_logger:main',
            'cmd_uart = py_tr24_car_ros.cmd_uart:main',
            'state_publisher = py_tr24_car_ros.state_publisher:main'
        ],
    },
)
