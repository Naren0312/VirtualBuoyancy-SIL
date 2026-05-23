import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rov_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Ament resource index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Package manifest
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Config / scenario files
        (os.path.join('share', package_name, 'config', 'scenarios'),
         glob(os.path.join('config', 'scenarios', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Naren',
    maintainer_email='naren@todo.todo',
    description='SIL simulator stack for Hammerhead ROV',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mock_dynamics_node = rov_sim.mock_dynamics_node:main',
            'scenario_runner = rov_sim.scenario_runner:main',
        ],
    },
)
