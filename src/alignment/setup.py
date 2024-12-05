from setuptools import find_packages, setup

package_name = 'alignment'

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
    maintainer='example',
    maintainer_email='example@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_detection_node = alignment.obstacle_detection:main',
            'motor_control_node = alignment.motor_control:main',
	        'is_aligned_node = alignment.is_aligned:main',
            'navigation = alignment.navigation:main'
        ],
    },
)
