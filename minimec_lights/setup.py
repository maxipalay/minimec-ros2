from setuptools import find_packages, setup

package_name = 'minimec_lights'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Maximiliano Palay',
    maintainer_email='maximilianopalay@gmail.com',
    description='Control for the minimec lights.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lights = minimec_lights.lights:main'
        ],
    },
)