from setuptools import setup
from glob import glob

package_name = 'morpheus_nav2'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],  # 对应同名目录（含 __init__.py）
    data_files=[
        # 必需：用于 ament 索引
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        # 必需：安装 package.xml
        ('share/' + package_name, ['package.xml']),
        # 安装所有 launch 脚本
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        # 安装所有配置（map.yaml/png、nav2_params.yaml、teleop_joy.yaml、twist_mux.yaml 等）
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    # 可执行脚本（不作为 Python 模块导入，直接装到可执行路径）
    # 注意：脚本第一行需有正确 shebang（#!/usr/bin/env python3 或 python3）
    scripts=[
        'scripts/gen_map_from_dem.py',
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='syh',
    maintainer_email='you@example.com',
    description='Nav2 integration for Morpheus (bringup, map, teleop & twist_mux configs).',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        # 如后续有 Python 可执行入口（在包内实现 main），可用 console_scripts 添加
        # 'console_scripts': [
        #     'some_node = morpheus_nav2.some_module:main',
        # ],
    },
)
