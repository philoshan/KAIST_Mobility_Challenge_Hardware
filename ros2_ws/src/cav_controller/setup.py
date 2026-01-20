from setuptools import setup

package_name = 'cav_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hy',
    maintainer_email='hy@todo.todo',
    description='CAV controller',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'run_drive = cav_controller.drive_test:main',
            'echo_pose = cav_controller.echo_pose:main',
            'p1_drive = cav_controller.p1:main',
        ],
    },
)