from setuptools import find_packages, setup

package_name = 'teraffe_drive_control_agent'

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
    maintainer='nuc',
    maintainer_email='seosang990911@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'robot_controller = teraffe_drive_control_agent.controllers.robot_controller:main',
            'ui = teraffe_drive_control_agent.ui:main',
            'ak80_64_status_ui = teraffe_drive_control_agent.ak80_64_status_ui:main',
            'encoder_status_ui = teraffe_drive_control_agent.encoder_status_ui:main',
            'dynamixel_status_ui = teraffe_drive_control_agent.dynamixel_status_ui:main',
            'dxl_encoder_status_ui = teraffe_drive_control_agent.dxl_encoder_status_ui:main',
            'robot_hardware_status_ui = teraffe_drive_control_agent.robot_hardware_status_ui:main',
            'robot_hardware_web_ui = teraffe_drive_control_agent.robot_hardware_web_ui:main',
        ],
    },
)
