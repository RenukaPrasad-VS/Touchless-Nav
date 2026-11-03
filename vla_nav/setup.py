from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'vla_nav'
model_data = []
for model_file in glob('models/**/*', recursive=True):
    if os.path.isfile(model_file):
        destination = os.path.join(
            'share', package_name, os.path.dirname(model_file))
        model_data.append((destination, [model_file]))

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/vla_nav.launch.py', 'launch/vla_sim.launch.py']),
        ('share/' + package_name + '/worlds', ['worlds/vla_objects.sdf']),
        ('share/' + package_name + '/config', ['config/vla_nav.yaml']),
        *model_data,
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Renuka Prasad VS',
    maintainer_email='renuka@example.com',
    description='Vision-language object grounding and Nav2 navigation for TurtleBot3.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'vla_object_nav = vla_nav.vla_node:main',
        ],
    },
)
