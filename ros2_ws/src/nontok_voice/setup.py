from glob import glob

from setuptools import find_packages, setup

package_name = 'nontok_voice'


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
        ('share/' + package_name + '/sounds', files('sounds/*')),
        ('share/' + package_name + '/system', files('system/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nontok team',
    maintainer_email='todo@example.com',
    description='Wake word detection.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wake_node = nontok_voice.wake_node:main',
            'trigger = nontok_voice.trigger:main',
            'listen = nontok_voice.listen:main',
            'speech_node = nontok_voice.speech_node:main',
            'eval_wake = nontok_voice.eval_wake:main'
        ],
    },
)
