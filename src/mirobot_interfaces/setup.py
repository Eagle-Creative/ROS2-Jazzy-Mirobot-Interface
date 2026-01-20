from setuptools import setup
import os
from glob import glob

package_name = 'mirobot_interfaces'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    py_modules=['scripts.mirobot_gui', 'scripts.mirobot_controller', 'scripts.mirobot_std_functions'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jonas',
    maintainer_email='jonas@example.com',
    description='Mirobot interfaces package with custom messages and nodes.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mirobot_gui = scripts.mirobot_gui:main',
            'mirobot_controller = scripts.mirobot_controller:main',
            'mirobot_std_functions = scripts.mirobot_std_functions:main',
        ],
    },
)