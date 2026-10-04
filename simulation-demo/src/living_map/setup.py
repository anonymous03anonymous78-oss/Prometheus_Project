import os
from glob import glob

from setuptools import setup

package_name = 'living_map'


def tree(src, dst):
    """data_files entries for every file below src (models keep their folder layout)."""
    out = []
    for root, _, files in os.walk(src):
        if files:
            out.append((os.path.join(dst, os.path.relpath(root, src)).rstrip('/.'),
                        [os.path.join(root, f) for f in files]))
    return out


setup(
    name=package_name,
    version='3.2.0',
    packages=[package_name],
    package_data={package_name: ['memory_seed.json', 'web/*']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
        ('share/' + package_name + '/web', glob('living_map/web/*')),
        ('share/' + package_name, glob('living_map/memory_seed.json')),
    ] + tree('models', 'share/' + package_name + '/models'),
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='Anonymous',
    maintainer_email='anonymous@example.com',
    description='The Living Map - TSYP14 Phase 1 simulation (v3)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'writer_node = living_map.writer_node:main',
            'executor_node = living_map.executor_node:main',
            'flix_node = living_map.flix_node:main',
            'beacon_network = living_map.beacon_network:main',
            'outside_network = living_map.outside_network:main',
            'command_post = living_map.command_post:main',
            'environment = living_map.environment:main',
            'gz_actions = living_map.gz_actions:main',
            'director_node = living_map.director_node:main',
            'dataflow_view = living_map.dataflow_view:main',
            'dry_run = living_map.dry_run:main',
        ],
    },
)
