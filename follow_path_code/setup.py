import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'follow_path_code'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='e.babenko',
    maintainer_email='e.babenko@g.nsu.ru',
    description='Autorace 2023',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "follow_node = follow_path_code.follow_node:main"
        ],
    },
)
