from setuptools import find_packages, setup

package_name = 'media_to_pybullet'

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
    maintainer='user',
    maintainer_email='alianlbj23@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mediapipe_node = media_to_pybullet.main:main',
        ],
    },
)
#ros2 run your_package_name your_node_name --ros-args -p rosbridge_ip:=192.168.1.100
