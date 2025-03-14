from setuptools import find_packages, setup

package_name = 'kuka_test_servo'

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
    maintainer='joe',
    maintainer_email='joe.liechty6@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "joint_servo_pub = kuka_test_servo.joint_servo_pub:main",
            "twist_servo_pub = kuka_test_servo.twist_servo_pub:main"
        ],
    },
)
