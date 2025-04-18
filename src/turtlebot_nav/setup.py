from setuptools import find_packages, setup

package_name = 'turtlebot_nav'

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
    maintainer='maryam',
    maintainer_email='maryam.harakat@epfl.ch',
    description='Basic Task',
    license='Apach License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'ball_tracker = turtlebot_nav.ball_tracker:main',
        ],
    },
)
