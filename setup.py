from glob import glob
from setuptools import find_packages, setup


setup(
    name='bimanual_manipulation',
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/bimanual_manipulation']),
        ('share/bimanual_manipulation', ['package.xml']),
        ('share/bimanual_manipulation/launch', glob('launch/*.launch.py')),
        ('share/bimanual_manipulation/config', glob('config/*.yaml')),
        ('share/bimanual_manipulation/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Howard',
    maintainer_email='howard@example.com',
    description='Minimal dual CRX and dual Sharpa Wave integration',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'hold_and_test = bimanual_manipulation.hold_and_test:main',
            'move_to_default_pose.py = bimanual_manipulation.move_to_default_pose:main',
            'tactile_wrench_viewer.py = bimanual_manipulation.tactile_wrench_viewer:main',
        ],
    },
)
