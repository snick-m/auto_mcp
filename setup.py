from setuptools import find_packages, setup

package_name = 'auto_mcp'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mushfiqur Rahman',
    maintainer_email='realmrahman.19@gmail.com',
    description='A package to introspect ROS2 topics and create an MCP server with tools interact with those topics.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hello = auto_mcp.hello:main',
            'introspect = auto_mcp.introspect:main',
            'server = auto_mcp.fast_server:main',
        ],
    },
)
