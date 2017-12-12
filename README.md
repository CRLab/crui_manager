# crui_manager

## Helpful Tutorials
- http://wiki.ros.org/ROS/Tutorials/Using%20a%20C%2B%2B%20class%20in%20Python

## Launch file
All of the following must be in the top level of your roslaunch due to a bug in moveit
```xml
<param name="analyze_grasp_topic" type="string" value="/crui_manager/analyze_grasp" />
<param name="execute_grasp_topic" type="string" value="/crui_manager/execute_grasp" />
<param name="run_recognition_topic" type="string" value="/crui_manager/run_recognition" />

<param name="grasp_approach_tran_frame" type="string" value="/approach_tran" />
<param name="world_frame" type="string" value="/world" />

<param name="arm_move_group_name" type="string" value="arm" />
<param name="gripper_move_group_name" type="string" value="gripper" />

<param name="analyzer_planner_id" type="string" value="[PRMkConfigDefault]" />
<param name="executor_planner_id" type="string" value="[BiRRTkConfigDefault]" />
<param name="allowed_analyzing_time" type="double" value="2" />
<param name="allowed_execution_time" type="double" value="8" />

<param name="final_block_position_x" type="double" value="0.2" />
<param name="final_block_position_y" type="double" value="0.3" />
<param name="final_block_position_z" type="double" value="0.2" />

<param name="pre_grasp_approach_direction_frame_id" type="string" value="/approach_tran" />
<param name="pre_grasp_approach_direction_x" type="double" value="0" />
<param name="pre_grasp_approach_direction_y" type="double" value="0" />
<param name="pre_grasp_approach_direction_z" type="double" value="1" />

<param name="post_grasp_retreat_direction_frame_id" type="string" value="/world" />
<param name="post_grasp_retreat_direction_x" type="double" value="0" />
<param name="post_grasp_retreat_direction_y" type="double" value="0" />
<param name="post_grasp_retreat_direction_z" type="double" value="1" />

<rosparam param="pre_grasp_goal_point_effort">[50.0, 50.0]</rosparam>
<rosparam param="pre_grasp_goal_point_positions">[0.0, 0.0]</rosparam>
<param name="pre_grasp_goal_point_time_from_start_secs" type="double" value="0" />
<rosparam param="pre_grasp_joint_names">[m1n6s200_joint_finger_1, m1n6s200_joint_finger_2]</rosparam>

<rosparam param="grasp_goal_point_effort">[50.0, 50.0]</rosparam>
<rosparam param="grasp_goal_point_positions">[1.2, 1.2]</rosparam>
<param name="grasp_goal_point_time_from_start_secs" type="double" value="0" />

<rosparam param="grasp_posture_joint_names">[m1n6s200_joint_finger_1, m1n6s200_joint_finger_2]</rosparam>

<param name="pre_grasp_approach_min_distance" type="double" value="0.05" />
<param name="pre_grasp_approach_desired_distance" type="double" value="0.1" />

<param name="post_grasp_retreat_min_distance" type="double" value="0.05" />
<param name="post_grasp_retreat_desired_distance" type="double" value="0.1" />

<param name="max_contact_force" type="double" value="-1" />

<node name="crui_manager" pkg="graspit_moveit_controller" type="crui_manager.py" output="screen" >
    <remap from="/joint_states"   to="/m1n6s200_driver/out/joint_state" />
</node>
```