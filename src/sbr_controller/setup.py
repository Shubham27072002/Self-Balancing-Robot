from setuptools import find_packages, setup

package_name = 'sbr_controller'

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
    maintainer='shubham',
    maintainer_email='27shubhamprajapati@gmail.com',
    description='ROS 2 controller for a simulated two-wheel self-balancing robot',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'balance_controller = sbr_controller.balance_controller:main',
            'balance_test = sbr_controller.balance_test:main',
        ],
    },
)
