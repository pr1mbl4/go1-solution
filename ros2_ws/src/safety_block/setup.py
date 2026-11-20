from setuptools import setup

package_name = 'safety_block'

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
    maintainer='Perspektiva',
    maintainer_email='perspektiva@example.com',
    description='Safety Block T2: /lowcmd_bare -> checks -> /lowcmd',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'safety_node = safety_block.safety_node:main',
        ],
    },
)
