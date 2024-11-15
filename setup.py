from setuptools import find_packages, setup
from glob import glob

package_name = 'spot_mine_navigation'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name + '/spot_mine_navigation_bringup/launch', glob('launch/*launch.[pxy][yma]*')),
        # ('share/' + package_name + '/worlds', glob('worlds/*.wbt')),
        # ('share/' + package_name + '/resource', glob('resource/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yixiang Gao',
    maintainer_email='ygao@missouri.edu',
    description='Using spot to automously navigate underground mine, a fully configured gazebo simulation is also included.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'thermal_publisher = spot_mine_navigation_application.src.thermal_image_publisher:main'
        ],
    },
)
