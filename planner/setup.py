from setuptools import setup, find_packages

package_name = 'planner'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    install_requires=[
        'setuptools',
        'rclpy',
    ],
    zip_safe=True,
    maintainer='cmos',
    maintainer_email='s24s1037hw@s.chibakoudai.jp',
    description='A ROS2 package for global and local planning',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'global_planner_node = global_planner.global_planner_node:main',
        ],
    },
)
