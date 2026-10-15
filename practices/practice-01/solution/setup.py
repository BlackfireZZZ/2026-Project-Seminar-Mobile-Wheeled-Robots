from glob import glob
from setuptools import find_packages, setup

package_name = 'practice01_digits'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Student',
    maintainer_email='student@example.com',
    description='Draw a two-digit number in turtlesim',
    license='MIT',
    entry_points={'console_scripts': [
        'prepare = practice01_digits.prepare:main',
        'draw_digit = practice01_digits.draw_digit:main',
    ]},
)
