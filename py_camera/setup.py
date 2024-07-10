from setuptools import find_packages, setup

package_name = 'py_camera'

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
    description='Acesso a camera',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_pub = py_camera.publisher_camera_function:main',
            'camera_pub2 = py_camera.publisher_camera_function2:main',
            'camera_sub = py_camera.subscriber_camera_function:main',
        ],
    },
)
