from setuptools import find_packages, setup

package_name = 'turtlebot_motion'

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
    maintainer='guelzim',
    maintainer_email='mohamed.guelzim@epfl.ch',
    description='Package for controlling the turtlebot motion',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'computation_controller = turtlebot_motion.computation_controller:main',
            'move_controller = turtlebot_motion.move_controller:main',
        ],
    },
)
