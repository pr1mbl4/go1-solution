from setuptools import setup

package_name = 'rl_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='RL Controller for T3 - Go1 Hackathon',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'rl_node = rl_controller.rl_node:main',
    ],
},
)
