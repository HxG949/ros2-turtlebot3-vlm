from setuptools import find_packages, setup

package_name = 'robot_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config',
            [
                'config/decision.yaml',
                'config/lidar_safety.yaml',
                'config/motion_controller.yaml',
                'config/parking_decision.yaml',
                'config/vlm_inference.yaml',
            ]),
        ('share/' + package_name + '/launch',
            [
                'launch/competition_parking.launch.py',
                'launch/semantic_navigation.launch.py',
            ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='guoxuehan',
    maintainer_email='18268163458@163.com',
    description=(
        'VLM perception, lidar safety, decision, and motion control nodes.'
    ),
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    # entry_points={
    #     'console_scripts': [
    #     ],
    # },
    entry_points={
        'console_scripts': [
            'decision_node = robot_perception.decision_node:main',
            'front_distance = robot_perception.front_distance:main',
            'lidar_safety_node = robot_perception.lidar_safety_node:main',
            'motion_controller_node = '
            'robot_perception.motion_controller_node:main',
            'parking_decision_node = '
            'robot_perception.parking_decision_node:main',
            'vlm_inference_node = robot_perception.vlm_inference_node:main',
        ],
    },
)
