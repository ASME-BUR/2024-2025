import os

# TODO Consider upgrading from this
from setuptools import find_packages, setup
from glob import glob

# Iterate through all the files and subdirectories
# to build the data files array
#
# Might be necessary for game objects (keep for now)
def generate_data_files(share_path, dir):
    data_files = []

    for path, _, files in os.walk(dir):
        list_entry = (share_path + path, [os.path.join(path, f) for f in files if not f.startswith('.')])
        data_files.append(list_entry)

    return data_files

package_name = 'bur_gz'

# Commented out directories will be relevant later

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch/', glob('launch/*')),
        # ('share/' + package_name + '/config/', glob('config/*')),
        ('share/' + package_name + '/urdf/', glob('urdf/*')),
        # ('share/' + package_name + '/worlds/', glob('worlds/*')),
        # ('share/' + package_name + '/meshes/visual', glob('meshes/visual/*')),
        ('share/' + package_name + '/rviz/', glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='theprabab@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
