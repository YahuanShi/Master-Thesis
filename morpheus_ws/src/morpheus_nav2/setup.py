from setuptools import setup
from glob import glob
import os

package_name = 'morpheus_nav2'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],  # 有同名目录即可，即使只有 __init__.py
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/map_server_standalone.launch.py']),
        # 安装地图文件（可选，但建议）
        ('share/' + package_name + '/config', ['config/map.yaml', 'config/map.png']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='syh',
    maintainer_email='you@example.com',
    description='Nav2 integration for Morpheus (map server demo)',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        # 这里暂时不需要可执行脚本
    },
)
