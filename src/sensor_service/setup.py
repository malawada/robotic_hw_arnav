from setuptools import setup

package_name = 'sensor_service'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arnav',
    maintainer_email='malawade.a@gmail.com',
    description='Package for accessing 3-DOF sensors',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'service1 = sensor_service.sensor1_service:main',
            'service2 = sensor_service.sensor2_service:main',
            'client = sensor_service.sensor_client:main',
        ],
    },
)
