from setuptools import find_packages, setup

package_name = 'robotController'

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
    maintainer='mik',
    maintainer_email='m.ragnoli011@studenti.unibs.it',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_node = robotController.control_node:main',
            'camera_control = robotController.camera_control:main'
        ],
    },
)
