<h2>Commands to launch</h2>

<h3>Point 1</h3>
<code>roslaunch iiwa_gazebo iiwa_gazebo_circular_object.launch</code> (To run the circular object in Gazebo)<br>
<code>rosrun opencv_ros opencv_ros_node</code> (To run the circular objects recognition)<br>

<h3>Points (2a) and (2b)</h3>
<b><u>Warning:</u></b> Check that the controllers loaded in "iiwa_gazebo_aruco.launch" are the "VelocityJointInterface"!<br><br>

<code>roslaunch iiwa_gazebo iiwa_gazebo_aruco.launch</code> (To run the aruco in Gazebo)<br>
<code>roslaunch aruco_ros usb_cam_aruco.launch camera:=/iiwa/camera1</code> (To run the aruco recognition)<br>
<code>rosrun kdl_ros_control kdl_robot_vision_control ./src/iiwa_stack_cv/iiwa_description/urdf/iiwa14.urdf</code> (To run the controller)<br>

<h3>Point (2c)</h3>
<b><u>Warning:</u></b> Check that the controllers loaded in "iiwa_gazebo_aruco.launch" are the "EffortJointInterface"!<br><br>

<code>roslaunch iiwa_gazebo iiwa_gazebo_aruco.launch</code> (To run the aruco in Gazebo)<br>
<code>roslaunch aruco_ros usb_cam_aruco.launch camera:=/iiwa/camera1</code> (To run the aruco recognition)<br>
<code>rosrun kdl_ros_control kdl_robot_test ./src/iiwa_stack_cv/iiwa_description/urdf/iiwa14.urdf</code> (To run the controller)<br>
