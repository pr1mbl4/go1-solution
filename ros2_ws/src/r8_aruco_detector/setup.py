from setuptools import setup, find_packages

package_name = 'r8_aruco_detector'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='altana',
    maintainer_email='anulanova06@gmail.com',
    description='ArUco marker detection node for Unitree Go1',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'aruco_node = r8_aruco_detector.aruco_node:main',
        ],
    },
)
