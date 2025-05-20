from setuptools import find_packages, setup

package_name = 'rt_eval'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jakob Friedl',
    maintainer_email='friedl.jak@gmail.com',
    description='Package for timing evaluation of ROS2 messages',
    license='BSD 3',
    entry_points={
        'console_scripts': [
            # <executable_name> = <module_path>:<callable>
            'latency_eval = rt_eval.latency_eval:main',
        ],
    },
)
