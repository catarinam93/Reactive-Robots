from setuptools import find_packages, setup

package_name = 'tri_project'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/robot_launch.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/map_0.wbt']))
data_files.append(('share/' + package_name + '/resource', ['resource/my_robot.urdf']))
data_files.append(('share/' + package_name + '/resource', ['resource/my_robot2.urdf']))
data_files.append(('share/' + package_name + '/music', ['music/sound_mono_16bit.wav']))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user.name@mail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot1_driver = tri_project.robot1_driver:main',
            'robot2_driver = tri_project.robot2_driver:main',
            'obstacle_avoider = tri_project.obstacle_avoider:main',
            'obstacle_robot_avoider = tri_project.obstacle_robot_avoider:main',
        ],
    },
)