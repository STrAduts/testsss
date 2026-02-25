from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'robot_follow_target'

# 收集配置/启动文件
def get_data_files():
    data_files = [
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ]
    # 加入config和launch目录
    for dir_path in ['config', 'launch']:
        for file in glob(os.path.join(dir_path, '*')):
            data_files.append(('share/' + package_name + '/' + dir_path, [file]))
    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    data_files=get_data_files(),
    install_requires=['setuptools', 'rclpy', 'numpy'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@xxx.com',
    description='ROS2 Humble robot follow target with fixed distance',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'target_tracker = robot_follow_target.target_tracker:main',
        ],
    },
)