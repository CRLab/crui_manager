import rospy
import std_msgs.msg
import external_controller_msgs.srv
import external_controller_msgs.msg

import collections

# CRUIState = collections.namedtuple('CRUIState', ['name', 'options', 'transitions', 'menu_type', 'parent'])


class CRUIState:
    pass




class CRUIController:
    READY_STATUS = 'ready'
    LOADING_STATUS = 'loading'

    def __init__(self, crui_manager):
        self.crui_manager = crui_manager

        self.status_publisher = rospy.Publisher('/crui_bot_status', std_msgs.msg.String, queue_size=1)
        self.valid_commands_publisher = rospy.Publisher('/valid_commands', external_controller_msgs.msg.ValidCommands, queue_size=1)

        self.execute_command_subscriber = rospy.Subscriber('/execute_command', std_msgs.msg.String, self.execute_command)

        self.set_environment_service = rospy.Service(
            "set_environment_service",
            external_controller_msgs.srv.SetEnvironment,
            self.set_environment
        )

        self.set_environment_service = rospy.Service(
            "get_current_environments_service",
            external_controller_msgs.srv.CurrentEnvironments,
            self.get_current_environments
        )

        self.get_valid_commands_service = rospy.Service(
            "valid_commands_service",
            external_controller_msgs.srv.CurrentCommands,
            self.get_valid_commands
        )

        self.states = {
            'main menu': CRUIState(
                name='main menu',
                options=['next block', 'select block', 'rerun vision'],
                transitions={
                    'next block': 'main menu',
                    'select block': 'grasp selection',
                    'rerun vision': 'main menu'
                },
                menu_type='menu',
                parent=''
            ),
            'grasp selection': CRUIState(
                name='grasp selection',
                options=['next grasp', 'select grasp', 'back'],
                transitions={
                    'next grasp': 'grasp selection',
                    'select grasp': 'main menu',
                    'back': 'main menu'
                },
                menu_type='submenu',
                parent='main menu'
            ),
            'grasp execution': CRUIState(
                name='grasp execution',
                options=['STOP'],
                transitions={
                    'STOP': 'main menu'
                },
                menu_type='submenu',
                parent='main menu'
            )
        }

        self.current_state = self.states['main menu']
        self.current_status = CRUIController.READY_STATUS

        self.publish_status()
        self.publish_valid_commands()
        rospy.loginfo(self.__class__.__name__ + " is inited")

    def get_valid_commands(self, request):
        current_commands = external_controller_msgs.srv.CurrentCommandsResponse()
        current_commands.commands = self.current_state.options
        current_commands.parent = self.current_state.parent
        current_commands.menutype = self.current_state.menu_type

        return current_commands

    def publish_valid_commands(self):
        valid_commands = external_controller_msgs.msg.ValidCommands()
        valid_commands.commands = self.current_state.options
        valid_commands.parent = self.current_state.parent
        valid_commands.menutype = self.current_state.menu_type
        self.valid_commands_publisher.publish(valid_commands)

    def set_environment(self, request):
        return True

    def get_current_environments(self):
        current_environments = external_controller_msgs.srv.CurrentEnvironmentsResponse()
        current_environments.environments = ["experiment"]
        return current_environments

    def publish_status(self):
        self.status_publisher.publish(self.current_status)

    def execute_command(self, command):
        # type: (std_msgs.msg.String) -> ()
        command = command.data

        if command == 'STOP':
            rospy.logwarn("Stopping execution")
            # TODO: Add code for stopping...
            self.crui_manager.stop_execution()
            self.current_status = CRUIController.READY_STATUS
            self.publish_status()
            return

        if self.current_status == CRUIController.LOADING_STATUS:
            rospy.logerr("Currently executing another command. Ignoring requested command '{}'.".format(command))
            return

        self.current_status = CRUIController.LOADING_STATUS
        self.publish_status()

        if command not in self.current_state.transitions:
            rospy.logwarn("Command '{}' is not a transition in state '{}'. Ignoring command.".format(command, self.current_state.name))
            return

        success = self.handle_command(command)
        if success:
            self.current_state = self.states[self.current_state.transitions[command]]
            rospy.loginfo("Successfully transitioned to state '{}'".format(self.current_state.name))
        else:
            rospy.logerr("Could not execute command '{}' and failed to transition to state '{}'.".format(command, self.current_state.name))

        self.current_status = CRUIController.READY_STATUS
        self.publish_status()
        self.publish_valid_commands()

    def handle_command(self, command):
        if command == 'next block':
            self.crui_manager.next_block()
        elif command == 'select block':
            self.crui_manager.select_block()
        elif command == 'rerun vision':
            self.crui_manager.rerun_vision()
        elif command == 'next grasp':
            self.crui_manager.next_grasp()
        elif command == 'select grasp':
            self.crui_manager.select_grasp()
        elif command == 'back':
            self.crui_manager.back_from_select_grasp()
        elif command == 'STOP':
            self.crui_manager.stop_execution()
        else:
            rospy.logerr("Command '{}' is not an executable command.".format(command))
            return False

        return True
