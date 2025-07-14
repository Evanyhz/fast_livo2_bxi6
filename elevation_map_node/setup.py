from setuptools import setup, find_packages

package_name = 'elevation_map_node'

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
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Elevation map generator node in Python',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'elevation_map_generator = elevation_map_node.elevation_map_generator:main',
            'elevation_map_generator5 = elevation_map_node.elevation_map_generator5:main',
        ],
    },
) 