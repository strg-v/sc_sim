from setuptools import find_packages, setup

package_name = 'py_spacecraft_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'ament_index_python'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='vincentvibr@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spacecraft_state_publisher = py_spacecraft_control.state_space_publisher:main',
            'spacecraft_attitute_cont_pid = py_spacecraft_control.attitude_control_pid:main',
            'spacecraft_attitute_cont_so3 = py_spacecraft_control.attitude_control_so3:main',
        ],
    },
)
