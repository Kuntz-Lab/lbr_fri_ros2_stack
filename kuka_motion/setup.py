from setuptools import find_packages, setup

package_name = 'kuka_motion'

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
    description='Kuka robot motion control package',
    license='Apache-2.0',
    tests_require=['pytest'],
    # extras_require={
    #     'test': ['pytest'],
    # },
    entry_points={
        'console_scripts': [
            'move_to_pose = kuka_motion.move_to_pose:main',
            'pose_publisher = kuka_motion.pose_pub:main',
            'tf_tree_sub = kuka_motion.tf_tree_sub:main',
            'move_to_pose_server = kuka_motion.move_to_pose_server:main',
            'move_to_pose_client = kuka_motion.move_to_pose_client:main',
            'joint_servo_pub = kuka_motion.joint_servo_pub:main',
            'twist_servo_pub = kuka_motion.twist_servo_pub:main',
            'move_home_server = kuka_motion.move_home_server:main',
            'move_home_client = kuka_motion.move_home_client:main',
        ],
    },
)
