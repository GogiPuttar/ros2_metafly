from setuptools import find_packages, setup

package_name = 'metafly_high'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/high_level.launch.py']),
        ('share/' + package_name + '/config', ['config/basic.rviz']),
        ('share/' + package_name + '/config', ['config/PID.rviz']),
    ],
    install_requires=['setuptools', 'rclpy', 'metafly_interfaces'],
    zip_safe=True,
    maintainer='adityanair',
    maintainer_email='aditya.nair0123@gmail.com',
    description='TODO: Package description',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'high_level_basic = metafly_high.high_level_basic:main',
            'high_level_PID = metafly_high.high_level_PID:main',
            'logger = metafly_high.logger:main',
        ],
    },
)
