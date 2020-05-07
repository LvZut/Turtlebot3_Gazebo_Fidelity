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
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'block_drive = sim_fidelity_pkg.block_drive:main',
		        'block_scan = sim_fidelity_pkg.block_scan:main',
                'block_IMU = sim_fidelity_pkg.block_IMU:main',
                'Odom_Scan_Test = sim_fidelity_pkg.Odom_Scan_Test:main',
        ],
    },
)
