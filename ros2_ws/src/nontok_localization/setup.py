from glob import glob

from setuptools import find_packages, setup

package_name = 'nontok_localization'


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
    description='AprilTag detection, tag-based localisation and EKF configuration.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tag_detector = nontok_localization.tag_detector_node:main',
            'tag_localizer = nontok_localization.tag_localizer_node:main',
            'fake_tags = nontok_localization.fake_tags_node:main',
            'verify_tags = nontok_localization.verify_tags:main',
            'calibrate_camera = nontok_localization.calibrate_camera:main',
            'make_tags = nontok_localization.make_tags:main'
        ],
    },
)
