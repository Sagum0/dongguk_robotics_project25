from setuptools import find_packages, setup

package_name = 'test_robotics_proj'

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
    maintainer_email='jyw010704@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_ready = test_robotics_proj.motor_ready:main',
            'data_hub = test_robotics_proj.data_hub:main',
            'trajectory_planner = test_robotics_proj.trajectory_planner:main',
            'inverse_kinematics = test_robotics_proj.inverse_kinematics:main',
            'motor_executor = test_robotics_proj.motor_executor:main',
        ],
    },
)
