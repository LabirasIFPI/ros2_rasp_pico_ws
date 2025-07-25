from setuptools import find_packages, setup

package_name = 'server_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='labiras',
    maintainer_email='PauloRAraujoLeal@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'server_control = server_control.server_control:main',
            'server_control_main = server_control.server_control_main:main',
        ],
    },
)
