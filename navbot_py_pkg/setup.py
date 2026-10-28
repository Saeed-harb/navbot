from setuptools import find_packages, setup

package_name = 'navbot_py_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy', 'ahrs'],
    zip_safe=True,
    maintainer='ts',
    maintainer_email='saeedharb00@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "data_plotter = navbot_py_pkg.data_plotter:main",
            "twist_stamper = navbot_py_pkg.twist_stamper:main",
            "robot_controller = navbot_py_pkg.robot_controller:main",
            "imu_publisher = navbot_py_pkg.imu_publisher:main",
            "path_follower_client = navbot_py_pkg.path_follower_client:main",
            "potentiometer_simulator = navbot_py_pkg.potentiometer_simulator:main",
            "pure_pursuit = navbot_py_pkg.pure_pursuit:main"
        ],
    },
)
