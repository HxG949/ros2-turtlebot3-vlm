from glob import glob

from setuptools import find_packages, setup


package_name = 'robot_acceptance'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='guoxuehan',
    maintainer_email='18268163458@163.com',
    description=(
        'Read-only monitoring and pure acceptance logic for P0 parking.'
    ),
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'robot_acceptance_monitor = '
            'robot_acceptance.monitor_node:main',
            'robot_acceptance_run = robot_acceptance.cli:run_main',
            'robot_acceptance_report = robot_acceptance.reporting:main',
        ],
    },
)
