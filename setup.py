from setuptools import setup
import os

package_name = 'dynamics_control'

data_files = [
    ('share/ament_index/resource_index/packages', [os.path.join('resource', package_name)]),
    ('share/' + package_name, ['package.xml']),
]

# include launch file from launch/ directory
if os.path.exists(os.path.join('launch', 'launch_dynamics.py')):
    data_files.append((os.path.join('share', package_name, 'launch'), [os.path.join('launch', 'launch_dynamics.py')]))

setup(
    name=package_name,
    version='0.0.1',
    py_modules=['dynamics_control'],
    install_requires=['setuptools', 'numpy', 'matplotlib', 'scipy'],
    data_files=data_files,
    entry_points={
        'console_scripts': [
            'unified_dynamics_node = dynamics_control:main'
        ],
    },
)
