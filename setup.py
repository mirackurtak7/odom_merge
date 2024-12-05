from setuptools import setup

package_name = 'odom_merge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/odom_merge_launch.py']),
        ('share/' + package_name + '/params', ['params/ekf.yaml']),  # params dosyasını ekliyoruz
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mirac Kurtak',
    maintainer_email='mirackurtak7@gmail.com',
    description='A package for merging odometry data',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'encoder_odom = odom_merge.encoder_odom:main',
            'lidar_odom = odom_merge.lidar_odom:main',
            'encoder = odom_merge.encoder:main',
        ],
    },
)

