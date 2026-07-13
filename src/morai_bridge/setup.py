from setuptools import setup

package_name = 'morai_bridge'

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
    maintainer='user',
    description='MORAI <-> ADAS-stack ROS2 topic adapter nodes',
    license='TODO',
    entry_points={
        'console_scripts': [
            'state_adapter_node = morai_bridge.state_adapter_node:main',
            'control_adapter_node = morai_bridge.control_adapter_node:main',
        ],
    },
)
