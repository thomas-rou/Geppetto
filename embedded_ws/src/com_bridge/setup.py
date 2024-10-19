from setuptools import find_packages, setup

package_name = 'com_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/com_bridge_launch.py']), 
        ('share/' + package_name + '/launch', ['launch/com_bridge_launch_gazebo.py']), 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='ely-cheikh.abass@polymtl.ca',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_controller = com_bridge.mission_server:main',
            'identify_robot = com_bridge.identify:main',
            'mission_status_manager = com_bridge.mission_status_manager:main'
        ],
    },
)
