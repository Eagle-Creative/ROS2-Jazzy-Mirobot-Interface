from setuptools import setup

package_name = 'mirobot_interfaces'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    py_modules=['scripts.mirobot_gui'],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jonas',
    maintainer_email='jonas@example.com',
    description='Mirobot interfaces package with custom messages and nodes.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mirobot_gui = scripts.mirobot_gui:main',
        ],
    },
)