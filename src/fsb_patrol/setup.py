from glob import glob

from setuptools import find_packages, setup

package_name = 'fsb_patrol'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.*')),
        ('share/' + package_name + '/config', glob('config/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sinan Emre',
    maintainer_email='186585487+sinann-emree@users.noreply.github.com',
    description='Patrol waypoint manager using Nav2',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'patrol_node = fsb_patrol.patrol_node:main',
        ],
    },
)
