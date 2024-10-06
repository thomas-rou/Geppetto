from setuptools import find_packages, setup

package_name = 'mission_status_manager'

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
    maintainer='loic-wsl',
    maintainer_email='loic.nguemegne-temena@polymtl.ca',
    description='TODO: Package description',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_manager = mission_status_manager.mission_manager:main'
        ],
    },
)
