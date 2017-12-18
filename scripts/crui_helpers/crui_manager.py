import rospy
import moveit_msgs.msg
import geometry_msgs.msg
import block_recognition.msg
import visualization_msgs.msg
import graspit_interface.msg
import std_msgs.msg

import typing
import moveit_commander
import curpp
import world_manager
# import grid_sample_client
# import graspit_commander
import tf
# import rospkg
# import os
import pyquaternion
import numpy as np


def construct_graspit_grasp(position, orientation):
    graspit_grasp_msg = graspit_interface.msg.Grasp()
    pose = geometry_msgs.msg.Pose()
    pose.position = position
    pose.orientation = orientation
    graspit_grasp_msg.pose = pose
    return graspit_grasp_msg


def plan_grasps():
    position = geometry_msgs.msg.Point(x=0, y=0, z=0.080)

    normal_orientation = pyquaternion.Quaternion(x=0, y=0, z=0, w=1)
    rotated_orientation = pyquaternion.Quaternion(axis=[1, 0, 0], angle=np.pi/2)
    rotated_orientation = rotated_orientation * normal_orientation

    geom_orient = geometry_msgs.msg.Quaternion(x=normal_orientation[0], y=normal_orientation[1], z=normal_orientation[2], w=normal_orientation[3])
    geom_orient_rot = geometry_msgs.msg.Quaternion(x=rotated_orientation[0], y=rotated_orientation[1], z=rotated_orientation[2], w=rotated_orientation[3])

    grasps = list()
    grasps.append(construct_graspit_grasp(position, geom_orient))
    grasps.append(construct_graspit_grasp(position, geom_orient_rot))

    return grasps


def _create_marker(block, is_highlighted):
    # type: (block_recognition.msg.DetectedBlock) -> visualization_msgs.msg.Marker
    marker = visualization_msgs.msg.Marker()

    marker.header = block.pose_stamped.header
    marker.type = visualization_msgs.msg.Marker.CUBE
    marker.action = visualization_msgs.msg.Marker.ADD
    marker.lifetime = rospy.Duration(0)  # Show block until it is deleted

    if is_highlighted:
        # Add scaling factor based on the size of the cube * 1.1 for highlighted
        marker.scale.x = block.edge_length * 1.3
        marker.scale.y = block.edge_length * 1.3
        marker.scale.z = block.edge_length * 1.3
        marker.color = CRUIManager.HIGHLIGHTED_BLOCK_COLOR
        marker.id = -1  # "highlighted"
    else:
        # Add scaling factor based on the size of the cube
        marker.scale.x = block.edge_length * 1.1
        marker.scale.y = block.edge_length * 1.1
        marker.scale.z = block.edge_length * 1.1
        marker.color = CRUIManager.NORMAL_BLOCK_COLOR
        marker.id = block.unique_id

    marker.pose = block.pose_stamped.pose

    return marker


class CRUIManager(object):
    # Cyan color
    HIGHLIGHTED_BLOCK_COLOR = std_msgs.msg.ColorRGBA(r=0.45490196078, g=0.6862745098, b=0.67843137254, a=1.0)
    # Burlywood color
    NORMAL_BLOCK_COLOR = std_msgs.msg.ColorRGBA(r=0.870588, g=0.721569, b=0.529412, a=1.0)
    # Green color
    GRASP_MARKER_COLOR = std_msgs.msg.ColorRGBA(r=0.63529411764, g=0.67058823529, b=0.34509803921, a=1.0)

    def __init__(self):

        # Initialize publishers
        self._recognized_blocks_publisher = rospy.Publisher("/ui_recognized_objects", visualization_msgs.msg.MarkerArray, queue_size=1)
        self._current_grasp_publisher = rospy.Publisher("/ui_current_grasp", visualization_msgs.msg.MarkerArray, queue_size=1)

        # Initialize action servers

        # Pull all params off param server
        self.grasp_approach_tran_frame = rospy.get_param("grasp_approach_tran_frame")
        self.world_frame = rospy.get_param("world_frame")
        self.arm_move_group_name = rospy.get_param("arm_move_group_name")
        self.gripper_move_group_name = rospy.get_param("gripper_move_group_name")

        self.analyzer_planner_id = rospy.get_param("analyzer_planner_id")
        self.executor_planner_id = rospy.get_param("executor_planner_id")
        self.allowed_analyzing_time = rospy.get_param("allowed_analyzing_time")
        self.allowed_execution_time = rospy.get_param("allowed_execution_time")

        # Initialize instance variables
        self._current_blocks = []
        self._highlighted_block = block_recognition.msg.DetectedBlock()
        self._highlighted_block_index = 0
        self._chosen_block = block_recognition.msg.DetectedBlock()

        self.grasp_markers = {}
        self.current_grasp_id = ""
        self._successful_grasps = []
        self._current_grasp = {}
        self._current_grasp_index = 0

        # Initialize ros service interfaces
        self.grasping_controller = curpp.MoveitPickPlaceInterface(
            arm_name=self.arm_move_group_name,
            gripper_name=self.gripper_move_group_name,
            grasp_approach_tran_frame=self.grasp_approach_tran_frame,
            analyzer_planner_id=self.analyzer_planner_id,
            execution_planner_id=self.executor_planner_id,
            allowed_analyzing_time=self.allowed_analyzing_time,
            allowed_execution_time=self.allowed_execution_time
        )

        self.scene = moveit_commander.PlanningSceneInterface()
        self.world_manager_client = world_manager.world_manager_client.WorldManagerClient()
        self.tf_listener = tf.TransformListener()
        # self._graspit_commander = graspit_commander.GraspitCommander()

        rospy.loginfo(self.__class__.__name__ + " is inited")

    def _graspit_grasp_to_moveit_grasp(self, object_name, graspit_grasp):
        # type: (str, graspit_interface.msg.Grasp) -> moveit_msgs.msg.Grasp

        pre_grasp_approach_direction = geometry_msgs.msg.Vector3Stamped()
        pre_grasp_approach_direction.header.frame_id = rospy.get_param("pre_grasp_approach_direction_frame_id")
        pre_grasp_approach_direction.vector.x = rospy.get_param("pre_grasp_approach_direction_x")
        pre_grasp_approach_direction.vector.y = rospy.get_param("pre_grasp_approach_direction_y")
        pre_grasp_approach_direction.vector.z = rospy.get_param("pre_grasp_approach_direction_z")

        post_grasp_retreat_direction = geometry_msgs.msg.Vector3Stamped()
        post_grasp_retreat_direction.header.frame_id = rospy.get_param("post_grasp_retreat_direction_frame_id")
        post_grasp_retreat_direction.vector.x = rospy.get_param("post_grasp_retreat_direction_x")
        post_grasp_retreat_direction.vector.y = rospy.get_param("post_grasp_retreat_direction_y")
        post_grasp_retreat_direction.vector.z = rospy.get_param("post_grasp_retreat_direction_z")

        moveit_grasp_msg = curpp.graspit_grasp_to_moveit_grasp(
            object_name=object_name,
            graspit_grasp_msg=graspit_grasp,
            listener=self.tf_listener,
            grasp_tran_frame_name=self.grasp_approach_tran_frame,
            end_effector_link=self.grasping_controller.get_end_effector_link(),

            pre_grasp_goal_point_effort=rospy.get_param("pre_grasp_goal_point_effort"),
            pre_grasp_goal_point_positions=rospy.get_param("pre_grasp_goal_point_positions"),
            pre_grasp_goal_point_time_from_start_secs=rospy.get_param("pre_grasp_goal_point_time_from_start_secs"),
            pre_grasp_joint_names=rospy.get_param("pre_grasp_joint_names"),

            grasp_goal_point_effort=rospy.get_param("grasp_goal_point_effort"),
            grasp_goal_point_positions=rospy.get_param("grasp_goal_point_positions"),
            grasp_goal_point_time_from_start_secs=rospy.get_param("grasp_goal_point_time_from_start_secs"),

            grasp_posture_joint_names=rospy.get_param("grasp_posture_joint_names"),

            pre_grasp_approach_min_distance=rospy.get_param("pre_grasp_approach_min_distance"),
            pre_grasp_approach_desired_distance=rospy.get_param("pre_grasp_approach_desired_distance"),
            pre_grasp_approach_direction=pre_grasp_approach_direction,

            post_grasp_retreat_min_distance=rospy.get_param("post_grasp_retreat_min_distance"),
            post_grasp_retreat_desired_distance=rospy.get_param("post_grasp_retreat_desired_distance"),
            post_grasp_retreat_direction=post_grasp_retreat_direction,

            max_contact_force=rospy.get_param("max_contact_force")
        )

        return moveit_grasp_msg

    def _analyze_grasp_reachability(self, object_name, graspit_grasp):
        # type: (str, graspit_interface.msg.Grasp) -> (moveit_msgs.msg.PickupResult, bool)
        """
        @return: Whether the grasp is expected to succeed
        """
        # Convert graspit grasp to moveit grasp
        rospy.loginfo("Analyzing grasp for object: {}".format(object_name))

        block_names = self.scene.get_attached_objects().keys()
        self.grasping_controller.detach_all_blocks(block_names)

        moveit_grasp_msg = self._graspit_grasp_to_moveit_grasp(object_name, graspit_grasp)
        success, pick_result = self.grasping_controller.analyze_moveit_grasp(object_name, moveit_grasp_msg)

        rospy.loginfo("Able to execute grasp with grasp id {} after analysis: {}".format(moveit_grasp_msg.id, success))

        return pick_result, success

    def _execute_grasp(self, object_name, graspit_grasp):
        # type: (str, graspit_interface.msg.Grasp) -> bool
        rospy.loginfo("Executing grasp goal")

        block_names = self.scene.get_attached_objects().keys()
        self.grasping_controller.detach_all_blocks(block_names)

        # Acquire block position for place
        objects = self.scene.get_object_poses([object_name])
        if object_name not in objects:
            rospy.logerr("Object {} not in planning scene. Execute grasp failed".format(object_name))
            return False

        block_pose_stamped = geometry_msgs.msg.PoseStamped()
        block_pose_stamped.pose = objects[object_name]
        block_pose_stamped.header.frame_id = self.grasping_controller.get_planning_frame()

        rospy.loginfo("Object {} in planning scene. Pose: {}".format(object_name, block_pose_stamped.pose))

        # Shift block pose to place location in param server
        block_pose_stamped.pose.position.x = rospy.get_param("final_block_position_x")
        block_pose_stamped.pose.position.y = rospy.get_param("final_block_position_y")
        block_pose_stamped.pose.position.z = rospy.get_param("final_block_position_z")

        # Convert graspit grasp to moveit grasp
        moveit_grasp_msg = self._graspit_grasp_to_moveit_grasp(object_name, graspit_grasp)

        # Execute pick on block
        success, pick_result = self.grasping_controller.execute_moveit_grasp(object_name, moveit_grasp_msg)
        # type: pick_result -> moveit_msgs.msg.PickupResult

        if not success:
            error_code = curpp.moveit_error_code_to_string(pick_result.error_code)
            rospy.logerr("Failed to execute pick. Reason: {}".format(error_code))
            return False
        else:
            rospy.loginfo("Successfully executed pick")

        rospy.loginfo("Placing block as position ({}, {}, {})"
                      .format(block_pose_stamped.pose.position.x,
                              block_pose_stamped.pose.position.y,
                              block_pose_stamped.pose.position.z))
        # Execute place on block
        success, place_result = self.grasping_controller.place(object_name, pick_result, block_pose_stamped)

        if not success:
            error_code = curpp.moveit_error_code_to_string(place_result.error_code)
            rospy.logerr("Failed to execute place. Reason: {}".format(error_code))
            return False
        else:
            rospy.loginfo("Successfully executed place")

        # Home arm and open hand
        success = self.grasping_controller.home_arm()
        if not success:
            rospy.logerr("Failed to home arm")
            return False
        else:
            rospy.loginfo("Successfully homed arm")

        success = self.grasping_controller.open_hand()
        if not success:
            rospy.logerr("Failed to open hand")
            return False
        else:
            rospy.loginfo("Successfully opened hand")

        return True

    def _run_recognition(self):
        rospy.loginfo("Running recognition")

        self.world_manager_client.clear_objects()

        detected_blocks = block_recognition.find_blocks()
        # type: detected_blocks -> typing.List[block_recognition.msg.DetectedBlock]

        if len(detected_blocks) == 0:
            rospy.loginfo("Detected no blocks. No work done.")
            return []

        rospy.loginfo("Detected {} blocks".format(len(detected_blocks)))

        for detected_block in detected_blocks:
            # Add all blocks to the scene
            self.world_manager_client.add_box(detected_block.unique_block_name,
                                              detected_block.pose_stamped,
                                              detected_block.edge_length,
                                              detected_block.edge_length,
                                              detected_block.edge_length)

            # Add blocks to graspit result
            self._current_blocks = detected_blocks

    def _remove_block_markers(self):
        marker_array = visualization_msgs.msg.MarkerArray()

        marker = visualization_msgs.msg.Marker()
        marker.action = visualization_msgs.msg.Marker.DELETEALL
        marker_array.markers.append(marker)

        self._recognized_blocks_publisher.publish(marker_array)

    def _publish_block_markers(self, publish_highlighted):
        marker_array = visualization_msgs.msg.MarkerArray()

        for block in self._current_blocks:
            marker_array.markers.append(_create_marker(block, is_highlighted=False))

        if publish_highlighted and self._highlighted_block is not None:
            marker_array.markers.append(_create_marker(self._highlighted_block, is_highlighted=True))

        self._recognized_blocks_publisher.publish(marker_array)

    def _publish_grasp_marker(self):
        self._remove_grasp_marker()
        if self._current_grasp.get("grasp_marker", None) is not None:
            grasp_marker = self._current_grasp.get("grasp_marker", None)
            self._current_grasp_publisher.publish(grasp_marker)

    def _remove_grasp_marker(self):
        grasp_marker = visualization_msgs.msg.MarkerArray()
        delete_all_marker = visualization_msgs.msg.Marker()
        delete_all_marker.action = visualization_msgs.msg.Marker.DELETEALL
        grasp_marker.markers.append(delete_all_marker)
        self._current_grasp_publisher.publish(grasp_marker)

    def next_block(self):
        if len(self._current_blocks) == 0:
            return

        # Highlight next block in list
        self._highlighted_block_index = (self._highlighted_block_index + 1) % len(self._current_blocks)
        self._highlighted_block = self._current_blocks[self._highlighted_block_index]

        # Publish recognized block markers with highlighted marker
        self._publish_block_markers(publish_highlighted=True)

    def select_block(self):
        rospy.loginfo("Entering select_block")
        # Assign currently selected block to variable
        if self._highlighted_block is None:
            rospy.logwarn("No block to select")
            return

        self._chosen_block = self._highlighted_block

        # Load block file from iv
        # rospack = rospkg.RosPack()
        # package_path = rospack.get_path('crui_manager')
        # block_path = os.path.join(package_path, 'resources', self._chosen_block.mesh_filename + ".xml")

        # Plan n grasps on block
        # self._graspit_commander.clearWorld()
        # self._graspit_commander.importGraspableBody(block_path)
        # self._graspit_commander.importRobot("MicoGripper")

        # resolution, sampling_type (1 is above sampler)
        rospy.loginfo("Computing grasps on object '{}' with mesh '{}'".format(self._chosen_block.unique_block_name, self._chosen_block.mesh_filename))
        # pre_grasps = grid_sample_client.GridSampleClient.computePreGrasps(resolution=10, sampling_type=1)
        # grasps = grid_sample_client.GridSampleClient.evaluatePreGrasps(pre_grasps.grasps)
        grasps = plan_grasps()

        # Create variable current_grasp_name = None
        current_grasp_name = "grasp0"
        grasp_markers = {}

        # Create subscriber to /move_group/display_grasp_markers
        def listen_for_grasp_markers(markers):
            # type: (visualization_msgs.msg.MarkerArray) -> ()
            print("Received marker for grasp '{}'".format(current_grasp_name))
            for marker in markers.markers:
                marker.color = CRUIManager.GRASP_MARKER_COLOR
            grasp_markers[current_grasp_name] = markers
        grasp_marker_subscriber = rospy.Subscriber('/move_group/display_grasp_markers', visualization_msgs.msg.MarkerArray, listen_for_grasp_markers)

        # Analyze each grasp
        analyzed_grasps = []
        for index, grasp in enumerate(grasps):
            # Assign current_grasp_name to "grasp{}".format(index)
            current_grasp_dict = {}
            current_grasp_name = "grasp{}".format(index)
            current_grasp_dict["grasp_name"] = current_grasp_name
            current_grasp_dict["grasp"] = grasp

            # Analyze grasp
            pickup_result, success = self._analyze_grasp_reachability(self._chosen_block.unique_block_name, grasp)

            # Store successful results in analyzed_grasps
            analyzed_grasps.append(current_grasp_dict)

        # Make sure we captured n grasps from display_grasp_markers
        for i in range(4):
            if len(grasp_markers) == len(grasps):
                break
            rospy.sleep(rospy.Duration(1))

        # Assign grasp_markers to grasps
        print("Total of {} grasps were successful out of {}".format(len(analyzed_grasps), len(grasps)))
        self._successful_grasps = []
        for grasp in analyzed_grasps:
            grasp["grasp_marker"] = grasp_markers.get(grasp["grasp_name"], visualization_msgs.msg.MarkerArray())
            self._successful_grasps.append(grasp)

        # Publish current grasp marker for grasp0
        if len(self._successful_grasps) > 0:
            self._current_grasp_index = 0
            self._current_grasp = self._successful_grasps[self._current_grasp_index]
            self._publish_grasp_marker()

        # Unsubscribe from display_grasp_markers
        grasp_marker_subscriber.unregister()

    def rerun_vision(self):
        rospy.loginfo("Running recognition")

        # Clear current blocks from scene
        self._remove_block_markers()
        self.world_manager_client.clear_objects()

        # Call block recognition
        detected_blocks = block_recognition.find_blocks()
        # type: detected_blocks -> typing.List[block_recognition.msg.DetectedBlock]

        if len(detected_blocks) == 0:
            rospy.loginfo("Detected no blocks. No work done.")
            return

        rospy.loginfo("Detected {} blocks".format(len(detected_blocks)))

        # Add new blocks to scene
        for detected_block in detected_blocks:
            # Add all blocks to the scene
            self.world_manager_client.add_box(detected_block.unique_block_name,
                                              detected_block.pose_stamped,
                                              detected_block.edge_length,
                                              detected_block.edge_length,
                                              detected_block.edge_length)

            # Add blocks to graspit result
            self._current_blocks = detected_blocks

        # Choose first block to be highlighted
        self._highlighted_block_index = 0
        self._highlighted_block = self._current_blocks[self._highlighted_block_index]

        # Publish recognized block markers with highlighted marker
        self._publish_block_markers(publish_highlighted=True)

    def next_grasp(self):
        rospy.loginfo("Running next grasp")

        if len(self._successful_grasps) == 0:
            return

        # Select next grasp in the list
        self._current_grasp_index = (self._current_grasp_index + 1) % len(self._successful_grasps)
        self._current_grasp = self._successful_grasps[self._current_grasp_index]

        # Publish grasp marker
        self._publish_grasp_marker()

    def select_grasp(self):
        rospy.loginfo("Running select grasp")

        # Clear grasp markers from scene and list of grasps
        self._remove_grasp_marker()
        self._successful_grasps = []

        # Execute grasp
        success = self._execute_grasp(self._chosen_block.unique_block_name, self._current_grasp["grasp"])

        # Rerun vision
        self._chosen_block = None
        self._current_grasp = {}
        self.rerun_vision()

    def back_from_select_grasp(self):
        rospy.loginfo("Back from select grasp")

        # Clear grasps from scene
        self._remove_grasp_marker()
        self._successful_grasps = []
        self._current_grasp = {}
        self._current_grasp_index = 0

    def stop_execution(self):
        rospy.loginfo("Preempt execution")
        # Preempt execution of trajectory controller
        self.grasping_controller.stop_execution()

        # Clear grasps and blocks from scene
        self._remove_grasp_marker()
        self._successful_grasps = []
        self._current_grasp = {}
        self._current_grasp_index = 0

        # Rerun vision
        self.rerun_vision()
