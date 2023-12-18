<h2>Commands to launch</h2>

<h3>Point (1)</h3>
<code>roslaunch iiwa_gazebo iiwa_gazebo_circular_object.launch</code> (To run the circular object in Gazebo)<br>
<code>rosrun opencv_ros opencv_ros_node</code> (To run the circular objects recognition)<br>

<h3>Point (2a)</h3>

<code>roslaunch iiwa_gazebo iiwa_gazebo_aruco.launch</code> (To run the aruco in Gazebo and the _VelocityJointInterface_ controllers)<br>
<code>roslaunch aruco_ros usb_cam_aruco.launch camera:=/iiwa/camera1</code> (To run the aruco recognition)<br>
<code>rosrun kdl_ros_control kdl_robot_vision_control_2a ./src/iiwa_stack_cv/iiwa_description/urdf/iiwa14.urdf</code> (To run the controller)<br>

<h3>Point (2b)</h3>

<code>roslaunch iiwa_gazebo iiwa_gazebo_aruco.launch</code> (To run the aruco in Gazebo and the _VelocityJointInterface_ controllers)<br>
<code>roslaunch aruco_ros usb_cam_aruco.launch camera:=/iiwa/camera1</code> (To run the aruco recognition)<br>
<code>rosrun kdl_ros_control kdl_robot_vision_control_2b ./src/iiwa_stack_cv/iiwa_description/urdf/iiwa14.urdf</code> (To run the controller)<br>

<h3>Point (2c)</h3>

<code>roslaunch iiwa_gazebo iiwa_gazebo_effort_aruco.launch</code> (To run the aruco in Gazebo and the _EffortJointInterface_ controllers)<br>
<code>roslaunch aruco_ros usb_cam_aruco.launch camera:=/iiwa/camera1</code> (To run the aruco recognition)<br>
<code>rosrun kdl_ros_control kdl_robot_test ./src/iiwa_stack_cv/iiwa_description/urdf/iiwa14.urdf</code> (To run the controller)<br>


<h2>Stack description</h2>

<ul>
  <li><code>opencv_ros</code>
    <ul>
      <li><code>opencv_ros_node.cpp</code>: contains the Blob detector (<i>Point (1c)</i>)</li>
    </ul>
  </li>
  <li><code>kdl_robot_cv</code>
    <ul>
      <li><code>kdl_robot_vision_control_2a.cpp</code>: implements the alignment of the camera to the aruco with a certain offset (<i>Point (2a)</i>)</li>
      <li><code>kdl_robot_vision_control_2b.cpp</code>: implements the look-at-point task on 2D (<i>Point (2b)</i>)</li>
      <li><code>kdl_robot_test.cpp</code>: implements the trajectory merged with the vision-based task (<i>Point (2c)</i>). Notice that this point was just about merging the controller developed in the previous homework with the vision-based task. The only thing that changes is the desired posed, which was the variable <code>des_pose</code> with two fields:
        <ul>
          <li><code>des_pose.p</code>: it is still computed by the planner (<code>KDLPlanner::compute_trajectory</code>)</li>
          <li><code>des_pose.M</code>: it is computed in about row 300: <code>des_pose.M = robot.getEEFrame().M*Re</code>, where <code>Re</code> is computed as the matrix needed to align the end-effector frame to the aruco frame.</li>
        </ul>
    </ul>  
  </li>
</ul>
