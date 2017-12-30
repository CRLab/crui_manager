import rospy
import std_msgs.msg
import external_controller_msgs.srv
import external_controller_msgs.msg
import graspit_interface.msg
import visualization_msgs.msg

import abc
import state
import curpp.skills


def next_block(crui_state, skill_manager):
    # type: (state.CRUIState, curpp.skills.SkillManager) -> ()
    rospy.loginfo("Entering next_block")
    if len(crui_state.current_blocks) == 0:
        rospy.loginfo("No blocks to highlight")
        return

    # Highlight next block in list
    crui_state.cycle_block()

    # Publish recognized block markers with highlighted marker
    skill_manager.remove_all_block_markers()
    skill_manager.publish_block_markers(crui_state.current_blocks, highlighted_block=crui_state.highlighted_block)


def select_block(crui_state, skill_manager):
    # type: (state.CRUIState, curpp.skills.SkillManager) -> ()
    rospy.loginfo("Entering select_block")
    # Assign currently selected block to variable
    if crui_state.highlighted_block is None:
        rospy.logwarn("No block to select")
        return

    # Plan default grasps
    grasps = curpp.skills.plan_grasps()

    # Analyze each grasp
    crui_state.clear_grasps()
    for index, grasp in enumerate(grasps):
        grasp_name = "grasp{}".format(index)
        grasp_promise = curpp.skills.capture_grasp_marker()

        # Analyze grasp
        pickup_result, success = \
            skill_manager.analyze_grasp_reachability(crui_state.highlighted_block.unique_block_name, grasp)

        grasp_markers = grasp_promise.get()

        # Store successful results in analyzed_grasps
        if success:
            crui_state.successful_grasps.append(state.GraspWrapper(grasp, grasp_name, grasp_markers))

    rospy.loginfo("Total of {} grasps were successful out of {}".format(len(crui_state.successful_grasps), len(grasps)))
    crui_state.publish_update("Found {} reachable grasps".format(len(crui_state.successful_grasps)))

    # Publish current grasp marker for grasp0
    skill_manager.publish_grasp_marker(crui_state.selected_grasp.grasp_markers)


def rerun_vision(crui_state, skill_manager):
    # type: (state.CRUIState, curpp.skills.SkillManager) -> ()
    crui_state.current_blocks = skill_manager.run_recognition()

    # Publish recognized block markers with highlighted marker
    skill_manager.publish_block_markers(crui_state.current_blocks, highlighted_block=crui_state.highlighted_block)


def next_grasp(crui_state, skill_manager):
    # type: (state.CRUIState, curpp.skills.SkillManager) -> ()
    crui_state.cycle_grasp()

    skill_manager.remove_all_grasp_markers()
    skill_manager.publish_grasp_marker(crui_state.selected_grasp)


def select_grasp(crui_state, skill_manager):
    # type: (state.CRUIState, curpp.skills.SkillManager) -> ()
    rospy.loginfo("Running select grasp")

    # Clear grasp markers from scene
    skill_manager.remove_all_grasp_markers()

    # Execute grasp
    success = skill_manager.execute_grasp(crui_state.highlighted_block.unique_block_name, crui_state.selected_grasp)

    # Rerun vision and remove all grasps
    crui_state.clear_grasps()
    skill_manager.run_recognition()


def back_from_grasp_selection(crui_state, skill_manager):
    # type: (state.CRUIState, curpp.skills.SkillManager) -> ()
    pass


def next_place_location(crui_state, skill_manager):
    # type: (state.CRUIState, curpp.skills.SkillManager) -> ()
    pass


def select_place_location(crui_state, skill_manager):
    # type: (state.CRUIState, curpp.skills.SkillManager) -> ()
    pass


def back_from_select_place_location(crui_state, skill_manager):
    # type: (state.CRUIState, curpp.skills.SkillManager) -> ()
    pass


class CRUIMenu:

    def __init__(self, crui_state, skill_manager):
        # type: (state.CRUIState, curpp.skills.SkillManager) -> ()
        self.crui_state = crui_state
        self.skill_manager = skill_manager

    @abc.abstractmethod
    def execute_command(self, command_str):
        pass


class BlockSelectionMenu(CRUIMenu):

    def __init__(self, crui_state, skill_manager):
        CRUIMenu.__init__(self, crui_state, skill_manager)
        self.crui_state.commands = [state.NEXT_BLOCK, state.SELECT_BLOCK, state.RERUN_VISION]

        if len(self.crui_state.current_blocks) == 0:
            rerun_vision(self.crui_state, self.skill_manager)

    def execute_command(self, command_str):
        if command_str == state.NEXT_BLOCK:
            next_block(self.crui_state, self.skill_manager)
            return BlockSelectionMenu
        elif command_str == state.SELECT_BLOCK:
            select_block(self.crui_state, self.skill_manager)
            return GraspSelectionMenu
        elif command_str == state.RERUN_VISION:
            rerun_vision(self.crui_state, self.skill_manager)
            return BlockSelectionMenu
        else:
            raise ValueError("command_str must be: {}".format(self.crui_state.commands))


class GraspSelectionMenu(CRUIMenu):

    def __init__(self, state):
        CRUIMenu.__init__(self, state)


class PlaceLocationSelectionMenu(CRUIMenu):

    def __init__(self, state):
        CRUIMenu.__init__(self, state)


StartMenu = BlockSelectionMenu
