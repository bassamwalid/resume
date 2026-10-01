from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'automilestoneT18'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='team729-pi',
    maintainer_email='team729-pi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [

            # Capitalized executable names
            'Localization_Team18 = automilestoneT18.Localization_Team18:main',
            'Planner_Controller_Team18 = automilestoneT18.Planner_Controller_Team18:main',
            'Planning_Team18 = automilestoneT18.Planning_Team18:main',
            'PurePursuit_Team18 = automilestoneT18.PurePursuit_Team18:main',
            'CityPlanning = automilestoneT18.CityPlanning:main',
            'path_plotter_team18 = automilestoneT18.PathPlotter_Team18:main',

            # Lowercase executable names too, to avoid launch errors
            'localization_team18 = automilestoneT18.Localization_Team18:main',
            'planner_controller_team18 = automilestoneT18.Planner_Controller_Team18:main',
            'planning_team18 = automilestoneT18.Planning_Team18:main',
            'purepursuit_team18 = automilestoneT18.PurePursuit_Team18:main',
            'cityplanning = automilestoneT18.CityPlanning:main',
        ],
    },
)