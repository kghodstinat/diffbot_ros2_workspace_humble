from setuptools import setup
from glob import glob
package_name='diffbot_pi'
setup(name=package_name,version='0.1.0',packages=[package_name],data_files=[('share/ament_index/resource_index/packages',['resource/'+package_name]),('share/'+package_name,['package.xml']),('share/'+package_name+'/launch',glob('launch/*.launch.py')),('share/'+package_name+'/config',glob('config/*.yaml')),('share/'+package_name+'/urdf',glob('urdf/*.urdf')),('share/'+package_name+'/udev',glob('udev/*.rules'))],install_requires=['setuptools'],zip_safe=True,maintainer='Robot developer',maintainer_email='robot@example.com',description='Pi GPIO ROS2 diff drive',license='Apache-2.0',entry_points={'console_scripts':['base_controller=diffbot_pi.base_controller:main','sonar=diffbot_pi.sonar:main']})
