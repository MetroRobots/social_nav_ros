from setuptools import setup

package_name = 'social_nav_util'

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
    maintainer='David V. Lu!!',
    maintainer_email='davidvlu@gmail.com',
    description='Utilities for social navigation work',
    license='BSD 3-clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['visualize_pedestrians = social_nav_util.visualize_pedestrians:main']
    },
)
