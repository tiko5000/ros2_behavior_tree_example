from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent, LogInfo
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.event_handlers import OnProcessStart
from launch.event_handlers.on_shutdown import OnShutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.events import matches_node_name
from launch_ros.event_handlers import OnStateTransition

from lifecycle_msgs.msg import Transition

from pathlib import Path

import pdb

def generate_launch_description():
    package_dir = Path(get_package_share_directory('ros2_behavior_tree_example'))
    behavior_tree_dir = package_dir / 'behavior_trees'

    # limit choices so we can only have available files
    mode_choices = ["sequence", "reactive_sequence"]
    tree_choices = ["ping_pong.xml", "ping_pong_no_decorator.xml", "ping_pong_executor.xml"]
    enable_choices = ["True", "False"]

    node1_enable_arg = DeclareLaunchArgument("node1_enable", 
                                            default_value="True",
                                            choices=enable_choices,
                                            description="Enable Primary Node in case you want to launch separately")
    node2_enable_arg = DeclareLaunchArgument("node2_enable", 
                                            default_value="True",
                                            choices=enable_choices,
                                            description="Enable Secondary Node in case you want to launch separately")

    node1_mode_arg = DeclareLaunchArgument("node1_mode", 
                                            default_value="reactive_sequence", 
                                            choices=mode_choices, 
                                            description="Set trees to reactive sequence")
    node2_mode_arg = DeclareLaunchArgument("node2_mode", 
                                            default_value="sequence", 
                                            choices=mode_choices, 
                                            description="Set trees to standard sequence")

    node1_behaviortree_arg = DeclareLaunchArgument("node1_behaviortree", 
                                                    default_value="ping_pong_executor.xml", 
                                                    choices=tree_choices, 
                                                    description="Set behevior tree file to use desired nodes")
    node2_behaviortree_arg = DeclareLaunchArgument("node2_behaviortree", 
                                                    default_value="ping_pong_no_decorator.xml", 
                                                    choices=tree_choices, 
                                                    description="Set behavior tree file to use desired nodes")

    bool_choices = ["True", "False", "true", "false"]
    auto_activate_lifecycle_nodes_arg = DeclareLaunchArgument('auto_activate_lifecycle_nodes',
                                                              default_value='True',
                                                              choices=bool_choices,
                                                              description='Automatically activate lifecycle nodes.')

    node_name = 'primary_ping_pong'
    primary_ping_pong_node = LifecycleNode(
        package='ros2_behavior_tree_example',
        executable='behavior_tree_example',
        name=node_name,
        output='screen',
        namespace='',
        remappings=[('incoming_pong', 'secondary_to_primary'),('outgoing_ping', 'primary_to_secondary')],
        parameters=[{
            # "rate_hz" : 1.0, # is handled by generate_parameter_library
            # "num_republish": 5, # is handled by generate_parameter_library
            # "ping_starter" : True, # is handled by generate_parameter_library
            "behaviortree_file" : PathJoinSubstitution([str(behavior_tree_dir),
                                                        LaunchConfiguration("node1_mode"),
                                                        LaunchConfiguration("node1_behaviortree")])
        }]
    )

    # Register an event handler to change the lifecycle state to configure before the node starts
    configure_event_handler = RegisterEventHandler(
        condition=IfCondition(LaunchConfiguration("auto_activate_lifecycle_nodes")),
        event_handler=OnProcessStart(
            target_action=primary_ping_pong_node,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(primary_ping_pong_node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        )
    )

    # Register an event handler to change the lifecycle state to activate after the node starts
    activate_event_handler = RegisterEventHandler(
        condition=IfCondition(LaunchConfiguration("auto_activate_lifecycle_nodes")),
        event_handler=OnStateTransition(
            target_lifecycle_node=primary_ping_pong_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                LogInfo(msg="node 'behavior_tree_lifecycle_node' reached the 'inactive' state, 'activating'."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(primary_ping_pong_node),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    # Register an event handler to change the lifecycle state to shutdown before the node shuts down
    shutdown_event_handler = RegisterEventHandler(
        condition=IfCondition(LaunchConfiguration("auto_activate_lifecycle_nodes")),
        event_handler=OnShutdown(
            on_shutdown=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_node_name(node_name),
                        transition_id=Transition.TRANSITION_ACTIVE_SHUTDOWN,
                    ),
                ),
            ],
        )
    )

    secondary_ping_pong_node = Node(
        condition=IfCondition(LaunchConfiguration("node2_enable")),
        package='ros2_behavior_tree_example',
        executable='behavior_tree_example',
        name='secondary_ping_pong',
        output='screen',
        remappings=[('incoming_pong', 'primary_to_secondary'),('outgoing_ping', 'secondary_to_primary')],
        parameters=[{            
            "rate_hz" : 0.75,
            "num_republish": 4,
            "ping_starter" : False,
            "behaviortree_file" : PathJoinSubstitution([str(behavior_tree_dir),
                                                        LaunchConfiguration("node2_mode"),
                                                        LaunchConfiguration("node2_behaviortree")])
            }]
    )

    return LaunchDescription([
        auto_activate_lifecycle_nodes_arg,
        node1_enable_arg,
        node2_enable_arg,
        node1_mode_arg,
        node2_mode_arg,
        node1_behaviortree_arg,
        node2_behaviortree_arg,
        primary_ping_pong_node,
        secondary_ping_pong_node,
        configure_event_handler,
        activate_event_handler,
        shutdown_event_handler,
    ])