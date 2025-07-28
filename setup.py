from setuptools import find_packages, setup

package_name = 'teamcream'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/depthheadinghold.launch.yaml']),
        ('share/' + package_name + '/launch', ['launch/lanefollow.launch.yaml']),
        ('share/' + package_name + '/launch', ['launch/apriltag_follow.launch.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eugene',
    maintainer_email='eugenewang920@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "Dance = teamcream.Dance:main",
            "bluerov2_sensors = teamcream.bluerov2_sensors:main",
            "ArmClient = teamcream.ArmClient:main",
            "forward_move = teamcream.forward_move:main",
            "depthhold = teamcream.depthhold:main",
            "headingcontrol = teamcream.headingcontrol:main",
            "calcdepth = teamcream.calcdepth:main",
            "controlMsgPub = teamcream.controlMsgPub:main",
            "imagedection = teamcream.imagedection:main",
            "lane_following_r = teamcream.lane_following_r:main",
            "lane_following_y = teamcream.lane_following_y:main",
            "LaneFollowingOutput = teamcream.LaneFollowingOutput:main",
            "apriltag_detection = teamcream.apriltag_detection:main",
            "apriltag_output_pub = teamcream.apriltag_output_pub:main", 
            "apriltag_x_output = teamcream.apriltag_x_output:main",
            "apriltag_y_output = teamcream.apriltag_y_output:main",
            "apriltag_z_output = teamcream.apriltag_z_output:main",
            "apriltag_r_output = teamcream.apriltag_r_output:main",
        ],
    },
)
