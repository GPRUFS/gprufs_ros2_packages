from setuptools import find_packages, setup

package_name = 'py_calmaN'

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
    description='Pacotes para realizar a interface e comunicação com o Calma-N: Real e Simulação',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'walk = py_calmaN.walk:main',
            'lidar_test = py_calmaN.lidar_test:main',
            'encoder_test = py_calmaN.encoder_test:main',
            'walk_pub = py_calmaN.walk_pub:main',
        ],
    },
)
