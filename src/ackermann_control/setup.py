from setuptools import find_packages, setup

package_name = 'ackermann_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yaristeven',
    maintainer_email='steayari@espol.edu.ec',
    description='Pure Pursuit controller for an Ackermann robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'pure_pursuit = ackermann_control.pure_pursuit:main',
        ],
    },
)
