from glob import glob
import os
from setuptools import setup

package_name = 'go1_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml') + glob('config/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Evgeniya',
    maintainer_email='evg.gazarova@mail.ru',
    description='Nav2 and SLAM for Go1',
    license='MIT',
    entry_points={'console_scripts': ['goal_relay = go1_nav.goal_relay:main']},
)
