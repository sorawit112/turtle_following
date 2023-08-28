from setuptools import find_packages, setup

package_name = 'turtle_control'

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
    maintainer='sorawit-i',
    maintainer_email='sorawit.i@obodroid.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_node = turtle_control.turtle:main',
            'random_walk_turtle = turtle_control.random_walk:main',
            'follow_turtle = turtle_control.follow_turtle:main'
        ],
    },
)
