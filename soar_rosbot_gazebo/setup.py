from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'soar_rosbot_gazebo'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'models', 'maze'),
            ['models/maze/model.config', 'models/maze/model.sdf']),
        (os.path.join('share', package_name, 'models', 'maze', 'meshes'),
            glob('models/maze/meshes/*.stl')),
        (os.path.join('share', package_name, 'models', 'maze', 'materials', 'scripts'),
            glob('models/maze/materials/scripts/*.material')),
        (os.path.join('share', package_name, 'models', 'maze', 'materials', 'textures'),
            glob('models/maze/materials/textures/*.jpg') + glob('models/maze/materials/textures/*.png')),
        (os.path.join('share', package_name, 'models', 'aruco1'),
            ['models/aruco1/model.config', 'models/aruco1/model.sdf']),
        (os.path.join('share', package_name, 'models', 'aruco1', 'meshes'),
            glob('models/aruco1/meshes/*.obj') + glob('models/aruco1/meshes/*.mtl')),
        (os.path.join('share', package_name, 'models', 'aruco1', 'textures'),
            glob('models/aruco1/textures/*.png') + glob('models/aruco1/textures/*.jpg')),
        (os.path.join('share', package_name, 'soar_rules'), glob('soar_rules/*.soar')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alvaro J. Gaona',
    maintainer_email='alvgaona@gmail.com',
    description='Soar Gazebo maze environment with ROSbot',
    license='MIT',
    tests_require=['pytest'],
)
