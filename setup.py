from setuptools import find_packages, setup

package_name = 'tenx_assignment'

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
    maintainer='shareef',
    maintainer_email='shareef@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory_generator_node = tenx_assignment.nodes.trajectory_generator_node:main',
            'trajectory_generator_node_test = tenx_assignment.nodes.trajectory_generator_node_test:main',
            'tracking_controller_node = tenx_assignment.nodes.tracking_controller_node:main',
            'path_publisher_node = tenx_assignment.nodes.path_publisher_node:main',
            'trajectory_to_path = tenx_assignment.nodes.trajectory_to_path:main',
            'path_smoother_node = tenx_assignment.nodes.path_smoother_node:main',
            'trajectory_controller_node = tenx_assignment.nodes.trajectory_controller_node:main',
        ],
    },
)
