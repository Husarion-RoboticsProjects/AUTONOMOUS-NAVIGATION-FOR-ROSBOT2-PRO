from setuptools import setup

package_name = 'obstacle_avoidance_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='husarion',
    maintainer_email='TODO',
    description='Obstacle avoidance for ROSbot using range sensors',
    license='TODO',
    entry_points={
        'console_scripts': [
            'obstacle_avoidance_node = obstacle_avoidance_pkg.obstacle_avoidance_node:main'
        ],
    },
)
