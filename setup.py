from setuptools import find_packages, setup

package_name = 'auto_mcp'

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
    maintainer='Mushfiqur Rahman',
    maintainer_email='realmrahman.19@gmail.com',
    description='TODO: Package description',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hello = auto_mcp.hello:main',
            'introspect = auto_mcp.introspect:main',
        ],
    },
)
