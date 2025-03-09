from setuptools import find_packages, setup
import os

package_name = 'sanitizer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), ['launch/sanitize_map.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='guglielmo',
    maintainer_email='guglielmo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['energy_map = sanitizer.energy_map:main', 'power_map = sanitizer.power_map:main', 'sanitizer = sanitizer.sanitizer_tsp_v2:main', 'room_detector = sanitizer.room_detector:main'],

    },
)
