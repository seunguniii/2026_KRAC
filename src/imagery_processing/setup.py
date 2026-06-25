from setuptools import find_packages, setup
import os
import glob

package_name = 'imagery_processing'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob(os.path.join('launch','*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sujin',
    maintainer_email='sujin@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'marker_recognition_x500 = imagery_processing.marker_recognition_x500:main',
            'marker_recognition_vtol = imagery_processing.marker_recognition_vtol:main',
            'following_recognition = imagery_processing.following_recognition:main'
        ],
    },
)
