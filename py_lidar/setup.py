from setuptools import find_packages, setup

package_name = 'py_lidar'

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
    maintainer='gprufs',
    maintainer_email='gprufs@gmail.com',
    description='Pacote para interface com o Lidar',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_pub = py_lidar.publisher_lidar_function:main', 
            'lidar_pub2 = py_lidar.publisher_lidar_function2:main',
            'lidar_sub = py_lidar.subscriber_lidar_function:main', 
        ],
    },
)
