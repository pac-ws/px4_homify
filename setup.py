import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'px4_homify'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yam]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='saurav',
    maintainer_email='sauravag@upenn.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_fix = px4_homify.gps_fix:main'
        ],
    },
)
