from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'soft_world'

setup(
    name=package_name,
    version='0.0.1',

    packages=find_packages(
        exclude=['test']
    ),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join(
                'share',
                package_name,
                'launch'
            ),
            glob('launch/*.launch.py')
        ),
        (
            os.path.join(
                'share',
                package_name,
                'scripts'
            ),
            glob('scripts/*.py')
        ),
        (
            os.path.join(
                'share',
                package_name,
                'usd'
            ),
            glob('usd/*.usd')
        ),
    ],

    install_requires=['setuptools'],

    zip_safe=True,

    maintainer='user',
    maintainer_email='user@example.com',

    description='Soft robot world for NVIDIA Isaac Sim',

    license='Apache-2.0',

    entry_points={
        'console_scripts': [],
    },
)
