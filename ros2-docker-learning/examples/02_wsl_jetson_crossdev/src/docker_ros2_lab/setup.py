from setuptools import setup
from glob import glob
import os
package_name = 'docker_ros2_lab'
setup(name=package_name, version='0.1.0', packages=[package_name],
      data_files=[('share/ament_index/resource_index/packages',['resource/'+package_name]),
                  ('share/'+package_name,['package.xml']),
                  (os.path.join('share',package_name,'launch'),glob('launch/*.launch.py'))],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='ROS 2 Student', maintainer_email='student@example.com',
      description='Hardware-free ROS 2 Docker lab', license='Apache-2.0',
      entry_points={'console_scripts': ['fake_lidar = docker_ros2_lab.fake_lidar:main',
            'scan_monitor = docker_ros2_lab.scan_monitor:main',
            'platform_info = docker_ros2_lab.platform_info:main',]})
