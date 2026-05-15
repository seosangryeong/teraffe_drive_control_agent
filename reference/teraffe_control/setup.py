from setuptools import find_packages, setup

package_name = 'teraffe_control'

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
            'teraffe_control_node = teraffe_control.teraffe_control:main',
            'panda_joint_state_filter = teraffe_control.franka_state_filter:main',
            'teraffe_control_imu = teraffe_control.teraffe_control_imu:main',
            'teraffe_control_imu_plot = teraffe_control.teraffe_control_imu_plot:main',
            'pandacontrol = teraffe_control.pandacontrol:main',
            'panda_rtb = teraffe_control.panda_rtb:main',
            'panda_ikpy = teraffe_control.panda_ikpy:main',
            'yz_plot = teraffe_control.yz_plot:main',
            'xyz_plot = teraffe_control.xyz_plot:main',
            'depth_pcl_visualizer = teraffe_control.depth_camera:main',
            'valve_ellipse_detector = teraffe_control.valve_depth_circle:main',
            'valve_ellipse_detector_zed = teraffe_control.valve_depth_zed:main',
            'valve_ellipse_detector_hough = teraffe_control.valve_depth_hough:main',
            'valve_ellipse_detector_realsense = teraffe_control.valve_depth_realsense:main',
            'valve_yolo12 = teraffe_control.valve_yolo12:main',
            'yolo_box = teraffe_control.yolo_box:main',
            'sam_test = teraffe_control.sam_test:main',
            'yolo_sam = teraffe_control.yolo_sam:main',
            'yolo_depth_plane = teraffe_control.valve_orientation:main',
            'valve_plane_circle = teraffe_control.valve_plane_circle:main',
            'valve_plane = teraffe_control.valve_plane:main'
        ],
    },
)
