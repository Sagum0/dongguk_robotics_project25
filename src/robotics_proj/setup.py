from setuptools import find_packages, setup

package_name = 'robotics_proj'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pc',
    maintainer_email='pc@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_ready = robotics_proj.motor_ready:main',
            'control_ready = robotics_proj.control_node:main',
            'test_control = robotics_proj.test_control:main',
        ],
    },
)
