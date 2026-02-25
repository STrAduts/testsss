from setuptools import setup
import os
from glob import glob

package_name = 'robot_follow_target'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],  # 对应Humble默认的Python代码目录
    data_files=[
        # Humble必需：包索引marker
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # package.xml
        ('share/' + package_name, ['package.xml']),
        # 配置文件（后续会创建）
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # 启动文件（后续会创建）
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Robot follow target with monocular camera (Humble)',
    license='Apache-2.0',
    tests_require=['pytest'],
    # 生成可执行节点（Humble下核心配置）
    entry_points={
        'console_scripts': [
            'camera_processor = robot_follow_target.camera_processor:main',
            'follow_controller = robot_follow_target.follow_controller:main',
        ],
    },
)