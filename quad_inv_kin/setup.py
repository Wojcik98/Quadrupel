from setuptools import setup

package_name = 'quad_inv_kin'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
    description='Inverse kinematics.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'leg_inv_kin_srv = {package_name}.leg_inv_kin:main',
            f'joy_steer = {package_name}.joy_steer:main',
        ],
    },
)
