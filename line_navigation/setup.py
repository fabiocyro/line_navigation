from setuptools import find_packages, setup

package_name = 'line_navigation'

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
    maintainer='Fabio Ribeiro',
    maintainer_email='fabio-cyro.ribeiro-batista@u-bourgogne.fr',
    description='Line Following for Turtlebot3 using ROS Humble',
    license='GPL-3.0 license',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'line_navigation_node = line_navigation.line_navigation:main',
        ],
    },
)
