from setuptools import setup

package_name = 'locomotion_core'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vboxuser',
    maintainer_email='vboxuser@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'movebase_core = locomotion_core.movebase_core:main',
            'drive_selector = locomotion_core.drive_selector:main',
            'cmd_roboteq = locomotion_core.cmd_roboteq:main',
            'rover_state_controller = locomotion_core.rover_state_controller:main',
            'rover_velocity = locomotion_core.rover_velocity:main',

        ],
    },
)
