import rospy
import menus
import state
import std_msgs.msg
import std_srvs.srv
import curpp.skills


class CRUIController:

    def __init__(self):
        self._crui_state = state.CRUIState()
        self._skill_manager = curpp.skills.SkillManager()
        self._current_menu = menus.StartMenu(self._crui_state, self._skill_manager)

        self._command_subscriber = rospy.Subscriber('/execute_command', std_msgs.msg.String, self._execute_command)
        self._republish_scene_service = rospy.Service('/rebroadcast_scene', std_srvs.srv.Empty, self._rebroadcast)

    def _execute_command(self, command_str):
        rospy.loginfo("Executing command {}".format(command_str))
        try:
            self._crui_state.publish_loading_status()
            next_menu = self._current_menu.execute_command(command_str)
            self._current_menu = next_menu(self._crui_state, self._skill_manager)
        except ValueError as e:
            rospy.loginfo("Unable to parse command {}".format(command_str))
        self._crui_state.publish_ready_status()

    def _rebroadcast(self, _):
        rospy.loginfo("Rebroadcasting scene")
        self._crui_state.publish_block_markers()
        self._crui_state.publish_grasp_marker()
        self._crui_state.publish_all_place_positions()
        self._crui_state.publish_current_status()
        self._crui_state.publish_valid_commands()

    def __del__(self):
        rospy.loginfo("Uninitalizing CRUIController")
        self._command_subscriber.unregister()
        self._republish_scene_service.shutdown()
