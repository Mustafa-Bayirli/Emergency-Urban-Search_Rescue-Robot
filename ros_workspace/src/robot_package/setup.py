from setuptools import setup
import os
from glob import glob

package_name = 'robot_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='daniel',
    maintainer_email='danielzhao2015@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gui = robot_package.gui:main',
            'driver = robot_package.driver:main',
            'HCSR04Wrapper = robot_package.HCSR04Wrapper:main',
            'AccelerometerWrapper = robot_package.AccelerometerWrapper:main',
            'TemperatureWrapper = robot_package.TemperatureWrapper:main'
        ],
    },
)
