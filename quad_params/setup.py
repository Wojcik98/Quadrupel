from setuptools import setup

package_name = 'quad_params'

setup(
    name=package_name,
    version='0.0.1',
    install_requires=['setuptools'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'quad_params.json']),
    ],
    zip_safe=True,
    author='Michał Wójcik',
    author_email="wojcikmichal98@gmail.com",
    maintainer='Michał Wójcik',
    maintainer_email="wojcikmichal98@gmail.com",
    keywords=['ROS', 'ROS2'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Service with robot parameters.',
    license='Apache License, Version 2.0',
)
