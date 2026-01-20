from setuptools import setup
import os
from glob import glob

package_name = 'cav_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # package index
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),

        # package.xml
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='...',
    maintainer_email='...',
    description='...',
    license='...',
    entry_points={
        'console_scripts': [
            'problem_1 = cav_controller.Problem_1:main',
        ],
    },
)
