from setuptools import setup
 
package_name = 'naive_crawl'
 
setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    data_files=[
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
    description='Naive crawling.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'naive_crawl = {package_name}.naive_crawl:main',
            f'transform_test = {package_name}.transform_test:main',
        ],
    },
)