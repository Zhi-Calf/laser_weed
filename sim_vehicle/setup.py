from setuptools import setup
import os
from glob import glob

package_name = 'sim_vehicle'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # RViz 预置视图：ros2 launch sim_vehicle weeder_sim.launch.py 会用到
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        # 一键启动
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jw11141',
    maintainer_email='jw11141@users.noreply.github.com',
    description='Simulation vehicle for laser-weeder integration testing',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'sim_vehicle = sim_vehicle.sim_vehicle_node:main',
            'sim_orchestrator = sim_vehicle.sim_orchestrator_node:main',
        ],
    },
)
