from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'Autonomous_Systems_Project_Team_18'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/Worlds', [
            'Worlds/Empty_Track.world',
            'Worlds/Racing_Track.world',
            'Worlds/City_Track.world',
        ]),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Team 18',
    maintainer_email='team18@todo.todo',
    description='Autonomous Systems Project — Team 18 (MS5)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [

            # ── MS2 ───────────────────────────────────────────────
            'OLR_node        = Autonomous_Systems_Project_Team_18.Autonomous_Systems_MS_2_OLR_Team_18:main',
            'Teleop_node     = Autonomous_Systems_Project_Team_18.Autonomous_Systems_MS_2_Teleop_Team_18:main',

            # ── MS3 / MS4 — Speed Controller ──────────────────────
            'Speed_Controller_node = Autonomous_Systems_Project_Team_18.Autonomous_Systems_MS_3_CLR_Alg_1_Speed_Team_18:main',

            # ── MS4 — Cmd Mixer ───────────────────────────────────
            'Cmd_Mixer_node  = Autonomous_Systems_Project_Team_18.Cmd_Mixer_Team_18:main',

            # ── MS4 — Lateral Controllers (one per track) ─────────
            'Lateral_Empty_node  = Autonomous_Systems_Project_Team_18.Lateral_Empty_Team_18:main',
            'Lateral_Racing_node = Autonomous_Systems_Project_Team_18.Lateral_Racing_Team_18:main',
            'Lateral_City_node   = Autonomous_Systems_Project_Team_18.Lateral_City_Team_18:main',

            # ── MS4 — Planning Nodes (one per track) ──────────────
            'Planning_Empty_node  = Autonomous_Systems_Project_Team_18.Planning_Empty_Team_18:main',
            'Planning_Racing_node = Autonomous_Systems_Project_Team_18.Planning_Racing_Team_18:main',
            'Planning_City_node   = Autonomous_Systems_Project_Team_18.Planning_City_Team_18:main',

            # ── MS5 — Localization Node (Kalman Filter) ───────────
            'Localization_node = Autonomous_Systems_Project_Team_18.Autonomous_Systems_MS_5_Localization_Team_18:main',
        ],
    },
)
