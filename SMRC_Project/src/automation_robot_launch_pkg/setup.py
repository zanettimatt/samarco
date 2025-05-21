from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'automation_robot_launch_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        #('share/ament_index/resource_index/packages',
        #    ['resource/' + package_name]),
        #('share/' + package_name, ['package.xml']),
        #(os.path.join('share', package_name, 'launch'), glob('launch/*')),

        # Install package.xml
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),

        # Install configuration files (if any)
        #(os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        #(os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='senai',
    maintainer_email='senai@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
