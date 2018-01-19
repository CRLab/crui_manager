import rospy
import std_msgs.msg
import curpp.skills
import curpp.constants
import external_controller_msgs.srv
import external_controller_msgs.msg
import block_recognition_msgs.msg
import graspit_interface.msg
import visualization_msgs.msg
import geometry_msgs.msg


# Constants
NEXT_BLOCK = 'next block'
SELECT_BLOCK = 'select block'
RERUN_VISION = 'rerun vision'

NEXT_GRASP = 'next grasp'
SELECT_GRASP = 'select grasp'
BACK_FROM_GRASP_SELECTION = 'back'

NEXT_POSITION = 'next position'
SELECT_POSITION = 'select position'
BACK_FROM_SELECT_POSITION = 'back'


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
        # Initialize publishers
        self._recognized_blocks_publisher = rospy.Publisher("/ui_recognized_objects", visualization_msgs.msg.MarkerArray, queue_size=1)
        self._grasp_marker_publisher = rospy.Publisher("/ui_current_grasp", visualization_msgs.msg.MarkerArray, queue_size=1)
        self._place_positions_publisher = rospy.Publisher("/ui_place_positions", visualization_msgs.msg.MarkerArray, queue_size=1)

        self.status_publisher = rospy.Publisher('/crui_bot_status', std_msgs.msg.String, queue_size=1)
        self.valid_commands_publisher = rospy.Publisher('/valid_commands', external_controller_msgs.msg.ValidCommands, queue_size=1)
        self.update_publisher = rospy.Publisher('/update_message', std_msgs.msg.String, queue_size=1)

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

        # Place positions
        self._place_positions = []
        self._selected_place_position_index = 0

    def get_valid_commands(self, request):
        current_commands = external_controller_msgs.srv.CurrentCommandsResponse()
        current_commands.commands = self.commands
        current_commands.parent = self.parent
        current_commands.menutype = self.menu_type

        return current_commands

    def publish_valid_commands(self):
        self.valid_commands_publisher.publish(self.get_valid_commands(None))

    def publish_loading_status(self):
        self.status_publisher.publish(CRUIState.LOADING_STATUS)

    def publish_ready_status(self):
        self.status_publisher.publish(CRUIState.READY_STATUS)

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
            return None
        return self._current_blocks[self._highlighted_block_index]

    @property
    def num_current_blocks(self):
        return len(self._current_blocks)

    def clear_blocks(self):
        self._current_blocks = []
        self._highlighted_block_index = 0

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
        self._selected_grasp_index = 0

    @property
    def num_current_grasps(self):
        return len(self._successful_grasps)

    # Place position
    def cycle_place_position(self):
        self._selected_place_position_index = (self._selected_place_position_index + 1) % len(self._place_positions)

    @property
    def place_positions(self):
        return self._place_positions

    @place_positions.setter
    def place_positions(self, place_positions):
        self._place_positions = place_positions
        self._selected_place_position_index = 0

    @property
    def selected_place_position(self):
        if len(self._place_positions) == 0:
            return None
        return self._place_positions[self._selected_place_position_index]

    @property
    def num_place_positions(self):
        return len(self._place_positions)

    def log(self, message):
        rospy.loginfo(message)
        self.update_publisher.publish(message)

    def clear_place_positions(self):
        self._place_positions = []
        self._selected_place_position_index = 0

    def remove_all_block_markers(self):
        self._recognized_blocks_publisher.publish(curpp.skills.generate_delete_all_marker_array())

    def publish_block_markers(self,
                              block_color=curpp.constants.NORMAL_BLOCK_COLOR,
                              highlighted_block_color=curpp.constants.HIGHLIGHTED_BLOCK_COLOR):

        marker_array = visualization_msgs.msg.MarkerArray()

        marker_array.markers = [
            curpp.skills.create_block_marker(block, is_highlighted=False, color=block_color)
            for block in self.current_blocks
        ]

        if self.highlighted_block is not None:
            marker_array.markers.append(curpp.skills.create_block_marker(self.highlighted_block,
                                        is_highlighted=True,
                                        color=highlighted_block_color))

        self._recognized_blocks_publisher.publish(marker_array)

    def publish_grasp_marker(self):
        self._grasp_marker_publisher.publish(self.selected_grasp.grasp_markers)

    def remove_all_grasp_markers(self):
        self._grasp_marker_publisher.publish(curpp.skills.generate_delete_all_marker_array())

    def publish_all_place_positions(self):
        place_position_markers = visualization_msgs.msg.MarkerArray()
        place_position_markers.markers = [curpp.skills.create_block_position_marker(self.highlighted_block, position) for position in self.place_positions]
        place_position_markers.markers.append(curpp.skills.create_block_position_marker(self.highlighted_block, self.selected_place_position, is_highlighted=True))
        self._place_positions_publisher.publish(place_position_markers)

    def remove_all_place_positions(self):
        self._place_positions_publisher.publish(curpp.skills.generate_delete_all_marker_array())
