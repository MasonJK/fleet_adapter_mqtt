#!/usr/bin/env python3

import glob
import os

from setuptools import find_packages, setup

package_name = 'fleet_adapter_mqtt'
share_dir = 'share/' + package_name
data_files_path = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    (share_dir, ['package.xml']),
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
    version='0.2.0',
    packages=[package_name],
    data_files=package_files(data_files_path, ['config/', 'doc/', 'map/', 'launch/']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Emima Jiva',
    maintainer_email='emji@ai2.upv.es',
    description='Robotnik MQTT RMF fleet adapter',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fleet_adapter=fleet_adapter_mqtt.fleet_adapter:main',
            'dispatch_action=fleet_adapter_mqtt.dispatch_action:main'
        ],
    },
)
