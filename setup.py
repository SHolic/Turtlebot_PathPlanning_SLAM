from setuptools import find_packages, setup

package_name = 'navigation_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='holic',
    maintainer_email='hitshl62412@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'turtlebot3_move_cam = navigation_pkg.turtlebot3_move_cam:main',
        	'image_processing_node = navigation_pkg.image_processing_node:main',
        	'map_update_node = navigation_pkg.map_update_node:main',
        	'path_planning_node = navigation_pkg.path_planning_node:main',
            'goal_commander_node = navigation_pkg.goal_commander_node:main',
        	'motion_control_node = navigation_pkg.motion_control_node:main',
        ],
    },
)
