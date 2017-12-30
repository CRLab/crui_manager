import rospy
import std_msgs.msg
import external_controller_msgs.srv
import external_controller_msgs.msg
import block_recognition_msgs.msg
import graspit_interface.msg
import visualization_msgs.msg


# Constants
NEXT_BLOCK = 'next block'
SELECT_BLOCK = 'select block'
RERUN_VISION = 'rerun vision'
SELECT_GRASP = 'select grasp'


class GraspWrapper:

    def __init__(self,
                 grasp=graspit_interface.msg.Grasp(),
                 grasp_name="",
                 grasp_markers=visualization_msgs.msg.MarkerArray()):

        # type: (graspit_interface.msg.Grasp, str, visualization_msgs.msg.MarkerArray) -> ()
        self.grasp = grasp
        self.grasp_name = grasp_name
        self.grasp_markers = grasp_markers


def set_environment(request):
    return True


def get_current_environments():
    current_environments = external_controller_msgs.srv.CurrentEnvironmentsResponse()
    current_environments.environments = ["experiment"]
    return current_environments


class CRUIState:
    READY_STATUS = 'ready'
    LOADING_STATUS = 'loading'

    MAIN_MENU_TYPE = 'menu'
    SUB_MENU_TYPE = 'submenu'

    def __init__(self):

        self.status_publisher = rospy.Publisher('/crui_bot_status', std_msgs.msg.String, queue_size=1)
        self.valid_commands_publisher = rospy.Publisher('/valid_commands', external_controller_msgs.msg.ValidCommands, queue_size=1)
        self.update_publisher = rospy.Publisher('/update_message', std_msgs.msg.String)

        self.set_environment_service = rospy.Service(
            "set_environment_service",
            external_controller_msgs.srv.SetEnvironment,
            set_environment
        )

        self.set_environment_service = rospy.Service(
            "get_current_environments_service",
            external_controller_msgs.srv.CurrentEnvironments,
            get_current_environments
        )

        self.get_valid_commands_service = rospy.Service(
            "valid_commands_service",
            external_controller_msgs.srv.CurrentCommands,
            self.get_valid_commands
        )

        self.commands = []
        self.parent = ''
        self.menu_type = CRUIState.MAIN_MENU_TYPE

        self.current_status = CRUIState.READY_STATUS

        # Blocks
        self._current_blocks = []
        self._highlighted_block_index = 0

        # Grasps
        self._successful_grasps = []
        self._selected_grasp_index = 0

    def get_valid_commands(self, request):
        current_commands = external_controller_msgs.srv.CurrentCommandsResponse()
        current_commands.commands = self.commands
        current_commands.parent = self.parent
        current_commands.menutype = self.menu_type

        return current_commands

    def publish_valid_commands(self):
        self.valid_commands_publisher.publish(self.get_valid_commands(None))

    def publish_status(self):
        self.status_publisher.publish(self.current_status)

    def publish_update(self, message):
        self.update_publisher.publish(message)

    # Blocks
    def cycle_block(self):
        self._highlighted_block_index = (self._highlighted_block_index + 1) % len(self.current_blocks)

    @property
    def current_blocks(self):
        return self._current_blocks

    @current_blocks.setter
    def current_blocks(self, current_blocks):
        self._current_blocks = current_blocks
        self._highlighted_block_index = 0

    @property
    def highlighted_block(self):
        if len(self._current_blocks) == 0:
            return block_recognition_msgs.msg.DetectedBlock()
        return self._current_blocks[self._highlighted_block_index]

    # Grasps
    def cycle_grasp(self):
        self._selected_grasp_index = (self._selected_grasp_index + 1) % len(self.successful_grasps)

    @property
    def successful_grasps(self):
        return self._successful_grasps

    @successful_grasps.setter
    def successful_grasps(self, successful_grasps):
        self._successful_grasps = successful_grasps
        self._selected_grasp_index = 0

    @property
    def selected_grasp(self):
        if len(self._successful_grasps) == 0:
            return GraspWrapper()
        return self._successful_grasps[self._selected_grasp_index]

    def clear_grasps(self):
        self._successful_grasps = []