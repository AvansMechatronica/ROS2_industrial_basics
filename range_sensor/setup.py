from setuptools import find_packages, setup

package_name = 'range_sensor'

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
    maintainer='developer',
    maintainer_email='44970086+GerardHarkemaAvans@users.noreply.github.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'template_publisher_script = range_sensor.template_publisher_script:main',
            'template_subscriber_script = range_sensor.template_subscriber_script:main',
            'sensor_info_publisher_simulation = range_sensor.sensor_info_publisher_simulation:main',
            'sensor_info_subscriber = range_sensor.sensor_info_subscriber:main',
            'metres_to_inches_server = range_sensor.metres_to_inches_server:main',
            'metres_to_inches_client = range_sensor.metres_to_inches_client:main',
            'counter_with_delay_action_server = range_sensor.counter_with_delay_action_server:main',
            'counter_with_delay_action_client = range_sensor.counter_with_delay_action_client:main',
            'assignment1 = range_sensor.assignment1:main',
            'assignment2 = range_sensor.assignment2:main',
        ],
    },
)
