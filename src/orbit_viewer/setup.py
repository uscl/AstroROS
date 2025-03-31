from setuptools import setup

package_name = 'orbit_viewer'

setup(
    name=package_name,
    version='0.15.2',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Mikael Arguedas',
    author_email='mikael@osrfoundation.org',
    maintainer='Aditya Pande, Shane Loretz',
    maintainer_email='aditya.pande@openrobotics.org, shane@openrobotics.org',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Examples of minimal subscribers using rclpy.',
    license='Apache License, Version 2.0',
    entry_points={
        'console_scripts': [
            'orbit_viewer ='
            ' orbit_viewer.orbit_viewer:main',
        ],
    },
)
