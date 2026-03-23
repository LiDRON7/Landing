from setuptools import find_packages, setup

package_name = 'lidar_preprocessing'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy', 'pcl', 'sensor_msgs_py'],
    zip_safe=True,
    maintainer='brian',
    maintainer_email='brianyariel.rm@gmail.com',
    description='LiDAR preprocessing package',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'lidar_preprocessing_node = lidar_preprocessing.lidar_preprocessing_node:main',
        ],
    },
)
