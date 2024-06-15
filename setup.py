from setuptools import find_packages, setup

package_name = 'vgraph_nav_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/turtlebot3_vgraph_navigation.launch.py',
            'launch/include/turtlebot3_amcl.launch.py',
            'launch/include/turtlebot3_world.launch.py',
            'launch/include/vgraph_planner.launch.py'
        ]),
        ('share/' + package_name + '/maps', [
            'maps/map.yaml',
            'maps/map.pgm'
        ]),
        ('share/' + package_name + '/rviz', ['rviz/turtlebot3_navigation.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='tenten31569@icloud.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vgraph_nav_ros2_node = vgraph_nav_ros2.vgraph_planner:main'
        ],
    },
)

