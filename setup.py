from setuptools import setup

package_name = 'ros2_bytes_transmit'

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
    maintainer='khinggan',
    maintainer_email='khinggan2013@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'server = ros2_bytes_transmit.server:main',
            'client= ros2_bytes_transmit.client:main',
        ],
    },
)
