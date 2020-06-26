from setuptools import setup

package_name = 'sim_fidelity_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='louis',
    maintainer_email='louisvincentvanzutphen@gmail.com',
    description='This package contains several scripts used for experimenting and testing the Turtlebot3 Burger bot sensors',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'Odom_Test = sim_fidelity_pkg.Odom_Test:main',
		        'LiDAR_Test = sim_fidelity_pkg.LiDAR_Test:main',
                'Gyro_Test = sim_fidelity_pkg.Gyro_Test:main',
                'Odom_Log = sim_fidelity_pkg.Odom_Log:main',
        ],
    },
)
