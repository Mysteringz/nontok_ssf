from glob import glob

from setuptools import find_packages, setup

package_name = 'nontok_base'


def files(pattern):
    return [path for path in glob(pattern, recursive=True) if '.' in path.split('/')[-1]]


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', files('config/*')),
        ('share/' + package_name + '/config/cameras', files('config/cameras/*')),
        ('share/' + package_name + '/launch', files('launch/*')),
        ('share/' + package_name + '/urdf', files('urdf/*')),
        ('share/' + package_name + '/models', files('models/*')),
        ('share/' + package_name + '/system', files('system/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nontok team',
    maintainer_email='todo@example.com',
    description='Serial bridge between ROS 2 and the Nontok Mega firmware.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'base_bridge = nontok_base.base_bridge_node:main',
            'fake_mega = nontok_base.fake_mega:main',
            'calibrate_odometry = nontok_base.calibrate_odometry:main'
        ],
    },
)
