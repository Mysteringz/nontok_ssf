from glob import glob

from setuptools import find_packages, setup

package_name = 'nontok_viz'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/web', glob('web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nontok team',
    maintainer_email='todo@example.com',
    description='Live dashboard: telemetry and camera views as a web page.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'viz_node = nontok_viz.viz_node:main',
        ],
    },
)
