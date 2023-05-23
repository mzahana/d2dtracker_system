from setuptools import setup

package_name = 'd2dtracker_system'

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
    maintainer='Mohamed Abdelkader',
    maintainer_email='mohamedashraf123@gmail.com',
    description='This ROS 2 package is used to setup and run the D2DTracker system on hardware (mostly Nvidia Jetson boards)',
    license='BSD3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
