from setuptools import find_packages, setup
from glob import glob
import os


package_name = 'autorace_camera'

data_files=[
     ('share/ament_index/resource_index/packages',
             ['resource/' + package_name]),
     ('share/' + package_name, ['package.xml']),
     (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py')))
   ]

def package_files(data_files, directory_list):

    paths_dict = {}

    for directory in directory_list:

        for (path, directories, filenames) in os.walk(directory):

            for filename in filenames:

                file_path = os.path.join(path, filename)
                install_path = os.path.join('share', package_name, path)

                if install_path in paths_dict.keys():
                    paths_dict[install_path].append(file_path)

                else:
                    paths_dict[install_path] = [file_path]

    for key in paths_dict.keys():
        data_files.append((key, paths_dict[key]))

    return data_files

setup(
 name=package_name,
 version='0.1.0',
 packages=find_packages(exclude=['test']),
 data_files=package_files(data_files, ['calibration/']),
 install_requires=['setuptools'],
 zip_safe=True,
 maintainer='Vitalii Kudinov',
 maintainer_email='v.kudinov@g.nsu.ru',
 description='AutoRace ROS2 package that processes the image',
 license='Apache 2.0',
 tests_require=['pytest'],
 entry_points={
     'console_scripts': [
             'image_compensation = autorace_camera.image_compensation:main',
             'image_projection = autorace_camera.image_projection:main',
             'core_node_mission = autorace_camera.core_node_mission:main'
     ],
   },
)
