from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'soar_rosbot_ros'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alvaro J. Gaona',
    maintainer_email='alvgaona@gmail.com',
    description='SOAR integration with ROSbot XL',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'soar_rosbot_ros = soar_rosbot_ros.soar_rosbot_ros:main'
        ],
    },
)
