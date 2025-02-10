from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_py_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include the launch directory
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='c',
    maintainer_email='c@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "py_node = my_py_pkg.my_first_node:main",
            "offb_node = my_py_pkg.offb_node:main",
            "pbvs_node = my_py_pkg.pbvs_node:main",
            "offb_posvel = my_py_pkg.offb_posvel:main",
            "offb_square = my_py_pkg.offb_trayectoria_cuadrada:main"
        ],
    },
)