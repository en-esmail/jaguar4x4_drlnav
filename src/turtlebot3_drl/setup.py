import glob
import os

from setuptools import find_packages
from setuptools import setup

package_name = 'turtlebot3_drl'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', 'turtlebot3_ddrl_stage1.launch.py'))),
        ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', 'turtlebot3_drl_stage2.launch.py'))),
        ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', 'turtlebot3_drl_stage3.launch.py'))),
        ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', 'turtlebot3_drl_stage4.launch.py'))),
        ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', 'turtlebot3_drl_stage5.launch.py'))),
        ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', 'turtlebot3_drl_stage6.launch.py'))),
        ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', 'turtlebot3_drl_farm.launch.py'))),
        #Farm environment resources
        ('share/' + package_name + '/farm_environment', glob.glob(os.path.join('farm_environment', '*.*'))),
        ('share/' + package_name + '/farm_environment/meshes', glob.glob(os.path.join('farm_environment', 'meshes', '**', '*.*'), recursive=True)),
        ('share/' + package_name + '/farm_environment/urdf', glob.glob(os.path.join('farm_environment', 'urdf', '**', '*.*'), recursive=True)),
        ('share/' + package_name + '/farm_environment/worlds', glob.glob(os.path.join('farm_environment', 'worlds', '**', '*.*'), recursive=True)),
        ('share/' + package_name + '/farm_environment/models', glob.glob(os.path.join('farm_environment', 'models', '**', '*.*'), recursive=True)),




    ],
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    author=['Gilbert', 'Ryan Shim'],
    author_email=['kkjong@robotis.com', 'jhshim@robotis.com'],
    maintainer='Pyo',
    maintainer_email='pyo@robotis.com',
    keywords=['ROS', 'ROS2', 'examples', 'rclpy'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'DDPG for TurtleBot3.'
    ),
    license='Apache License, Version 2.0',
    entry_points={
        'console_scripts': [
            'environment = turtlebot3_drl.drl_environment.drl_environment:main',
            'real_environment = turtlebot3_drl.drl_environment.drl_environment_real:main',
            'gazebo_goals = turtlebot3_drl.drl_gazebo.drl_gazebo:main',
            'train_agent = turtlebot3_drl.drl_agent.drl_agent:main_train',
            'test_agent = turtlebot3_drl.drl_agent.drl_agent:main_test',
            'real_agent = turtlebot3_drl.drl_agent.drl_agent:main_real',
            'train_real_agent = turtlebot3_drl.drl_agent.drl_agent:main_train_real',  # <--- add this
            'slip_logger = turtlebot3_drl.slip_logger:main',


        ],
    },
)

