from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'assignment2_rt'

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
    maintainer='richard',
    maintainer_email='albertrichard080@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ui_node = assignment2_rt.ui_node:main',
            'safety_node = assignment2_rt.safety_node:main',
        ],
    },
)
