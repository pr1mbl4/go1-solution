from setuptools import setup

package_name = 'odometry'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config',
            ['config/rtabmap_odom.yaml', 'config/ekf.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Xander',
    maintainer_email='xander@example.com',
    description='T4 odometry and simple driver for Go1',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_node = odometry.odom_node:main',
            'cmd_vel_test_publisher = odometry.cmd_vel_test_publisher:main',
        ],
    },
)