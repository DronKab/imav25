from setuptools import find_packages, setup
import os 
from glob import glob

package_name = 'imav25'

setup(
    name=package_name,
    version='0.0.0',
    packages=['imav25'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Copia los archivos launch
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        # --- ESTA ES LA PARTE CLAVE ---
        # Copia el modelo .blob y cualquier archivo en la carpeta tmr_model
        (os.path.join('share', package_name, 'resources/tmr_model'), 
         glob('resources/tmr_model/*.blob')),
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
            "keyboard = imav25.keyboard:main",
            "aruco_control = imav25.aruco_control:main",
            "goto_zone = imav25.goto_zone:main",
            "start_msg = imav25.start_msg:main", 
            "vision_yolo_node = imav25.vision_yolo_node:main",
            "fly_drone = imav25.fly_drone:main",
            "classes_publishers = imav25.classes_publishers:main",
            "ctrl_vision = imav25.ctrl_vision:main",
            "landing_node = imav25.landing_node:main",
            "indoor_smach = imav25.indoor_smach:main",
            "indoor_p1 = imav25.indoor_p1:main",
            "indoor_p2 = imav25.indoor_p2:main",
            "indoor_p3 = imav25.indoor_p3:main",
            "testing_smach = imav25.testing_smach:main",
            "ultimate_smach = imav25.ultimate_smach:main",
            "aruco_control_state = imav25.aruco_control_state:main"
        ],
    },
)