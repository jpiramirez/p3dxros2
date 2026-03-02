from setuptools import find_packages, setup

package_name = 'p3dxros2'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/robot_launch.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/p3dx.wbt']))
data_files.append(('share/' + package_name + '/resource', ['resource/p3dx.urdf']))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Juan-Pablo Ramirez-Paredes',
    maintainer_email='jpiramirez@gmail.com',
    description='A brief demostration of using the Pioneer 3-DX robot with Webots and ROS2',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'p3dxnode = p3dxros2.p3dxnode:main',
        ],
    },
)
