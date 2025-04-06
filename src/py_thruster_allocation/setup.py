from setuptools import find_packages, setup

package_name = 'py_thruster_allocation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['py_thruster_allocation/thruster_config.yaml']),

    ],
    install_requires=['setuptools', 'spacecraft_msgs', 'ament_index_python'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='vincentvibr@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'thruster_mapper_node = py_thruster_allocation.thruster_mapper_node:main'
        ],
    },
)
