from setuptools import find_packages, setup

package_name = 'servo_publisher'

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
    maintainer='siangting',
    maintainer_email='siangting200@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_trajectory_pub = servo_publisher.angle_pub:main',
            'servo_motor_pub = servo_publisher.motor_pub:main',
            'spider_walk = servo_publisher.spider_walk:main'
        ],
    },
)
