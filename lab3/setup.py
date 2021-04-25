import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'lab3'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name), glob('urdf/*'))
        ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kacper',
    maintainer_email='k.bugala@poczta.onet.pl',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nonkdl = lab3.nonkdl_dkin:main',
            'kdl = lab3.kdl_dkin:main'
        ],
    },
)
