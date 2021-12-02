import roslaunch
import rospy

world_name = "world_name:=$(find multi_robot_gazebo)/worlds/big_livingroom.world"
robot_num = "robot_num:=3"

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

vpp_cli_args = [
    "multi_robot_gazebo",
    "multi_robot_with_vpp.launch",
    world_name,
    robot_num,
]
vpp_roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(vpp_cli_args)[0]
vpp_roslaunch_args = vpp_cli_args[2:]

virtual_scan_server_cli_args = ["virtual_scan", "virtual_scan_server.launch"]
virtual_scan_server_roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(
    virtual_scan_server_cli_args
)[0]

occupancy_grid_server_cli_args = [
    "occupancy_grid_server",
    "occupancy_grid_server.launch",
]
occupancy_grid_server_roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(
    occupancy_grid_server_cli_args
)[0]

grnet_service_cli_args = ["grnet_detect", "grnet_service.launch"]
grnet_service_roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(
    grnet_service_cli_args
)[0]

PointCloud2ToObjectVecConverterServer_cli_args = [
    "pointcloud2_to_object_vec_converter",
    "PointCloud2ToObjectVecConverterServer.launch",
]
PointCloud2ToObjectVecConverterServer_roslaunch_file = (
    roslaunch.rlutil.resolve_launch_arguments(
        PointCloud2ToObjectVecConverterServer_cli_args
    )
)[0]


ViewPointExtractorServer_cli_args = [
    "view_point_extractor",
    "ViewPointExtractorServer.launch",
]
ViewPointEcxtractorServer_roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(
    ViewPointExtractorServer_cli_args
)[0]

launch_files = [
    (vpp_roslaunch_file, vpp_roslaunch_args),
    virtual_scan_server_roslaunch_file,
    occupancy_grid_server_roslaunch_file,
    grnet_service_roslaunch_file,
    PointCloud2ToObjectVecConverterServer_roslaunch_file,
    ViewPointEcxtractorServer_roslaunch_file,
]

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
launch = roslaunch.parent.ROSLaunchParent(uuid, launch_files)
launch.start()
rospy.loginfo("started")

# rospy.sleep()
# 3 seconds later
# launch.shutdown()
try:
    launch.spin()
finally:
    # After Ctrl+C, stop all nodes from running
    launch.shutdown()
