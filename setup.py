from setuptools import find_packages, setup
import os 
from glob import glob

package_name = 'imav25'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dronkab',
    maintainer_email='dronkab.unaq@gmail.com',
    description='TODO: Package description',
    license='MIT',
    entry_points={
        'console_scripts': [
            "px4_driver = imav25.px4_driver:main",
            "move_drone = imav25.move_drone:main",
            "take_photos = imav25.take_photos:main",
            "tunnel_detect = imav25.tunnel_detect:main",
            "platform_detect = imav25.platform_detect:main",
            "landing_platform = imav25.landing_platform:main",
            "cross_tunnel = imav25.cross_tunnel:main",
            "aruco_control = imav25.aruco_control:main",
            "save_pos = imav25.save_pos:main",
            "save_pos_image = imav25.save_pos_image:main",
            "whiteboard_detect = imav25.whiteboard_detect:main",
            "goto_whiteboard = imav25.goto_whiteboard:main"
        ],
    },
)