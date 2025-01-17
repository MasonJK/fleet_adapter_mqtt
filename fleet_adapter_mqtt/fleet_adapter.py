# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys
import argparse
import yaml
import nudged
import time
import threading
import datetime

import rclpy
import rclpy.node
from   rclpy.parameter import Parameter

import rmf_adapter as adpt
import rmf_adapter.vehicletraits as traits
import rmf_adapter.battery as battery
import rmf_adapter.geometry as geometry
import rmf_adapter.graph as graph
import rmf_adapter.plan as plan

from rmf_task_msgs.msg import TaskProfile, TaskType

from functools import partial

from .RobotCommandHandle import RobotCommandHandle
from .RobotClientAPI import RobotAPI
from .RobotDelivery import DeliveryTask

# ------------------------------------------------------------------------------
# Helper functions
# ------------------------------------------------------------------------------


def initialize_fleet(config_yaml, nav_graph_path, node, use_sim_time):
    # Profile and traits
    fleet_config = config_yaml['rmf_fleet']
    profile = traits.Profile(geometry.make_final_convex_circle(
        fleet_config['profile']['footprint']),
        geometry.make_final_convex_circle(fleet_config['profile']['vicinity']))
    vehicle_traits = traits.VehicleTraits(
        linear=traits.Limits(*fleet_config['limits']['linear']),
        angular=traits.Limits(*fleet_config['limits']['angular']),
        profile=profile)
    vehicle_traits.differential.reversible = fleet_config['reversible']

    # Battery system
    voltage = fleet_config['battery_system']['voltage']
    capacity = fleet_config['battery_system']['capacity']
    charging_current = fleet_config['battery_system']['charging_current']
    battery_sys = battery.BatterySystem.make(
        voltage, capacity, charging_current)

    # Mechanical system
    mass = fleet_config['mechanical_system']['mass']
    moment = fleet_config['mechanical_system']['moment_of_inertia']
    friction = fleet_config['mechanical_system']['friction_coefficient']
    mech_sys = battery.MechanicalSystem.make(mass, moment, friction)

    # Power systems
    ambient_power_sys = battery.PowerSystem.make(
        fleet_config['ambient_system']['power'])
    tool_power_sys = battery.PowerSystem.make(
        fleet_config['tool_system']['power'])

    # Power sinks
    motion_sink = battery.SimpleMotionPowerSink(battery_sys, mech_sys)
    ambient_sink = battery.SimpleDevicePowerSink(
        battery_sys, ambient_power_sys)
    tool_sink = battery.SimpleDevicePowerSink(battery_sys, tool_power_sys)

    nav_graph = graph.parse_graph(nav_graph_path, vehicle_traits)

    # Adapter
    fleet_name = fleet_config['name']
    node.get_logger().info(f"fleet name : {fleet_name}")
    adapter = adpt.Adapter.make(f'{fleet_name}_fleet_adapter')
    node.get_logger().info("made adapter")
    # if use_sim_time:
    #     adapter.node.use_sim_time()
    assert adapter, ("Unable to initialize fleet adapter. Please ensure "
                     "RMF Schedule Node is running")
    adapter.start()
    time.sleep(1.0)

    # TODO : not using server_uri
    fleet_handle = adapter.add_fleet(fleet_name, vehicle_traits, nav_graph)

    if not fleet_config['publish_fleet_state']:
        fleet_handle.fleet_state_publish_period(None)
    else:
        fleet_state_update_frequency = fleet_config['publish_fleet_state']
        fleet_handle.fleet_state_publish_period(
        datetime.timedelta(seconds=1.0/fleet_state_update_frequency))

    task_capabilities_config = fleet_config['task_capabilities']

    # Account for battery drain
    drain_battery = fleet_config['account_for_battery_drain']
    lane_merge_distance = fleet_config['lane_merge_distance']
    waypoint_merge_distance = fleet_config['waypoint_merge_distance']
    recharge_threshold = fleet_config['recharge_threshold']
    recharge_soc = fleet_config['recharge_soc']
    finishing_request = fleet_config['task_capabilities']['finishing_request']
    node.get_logger().info(f"Finishing request: [{finishing_request}]")
    # Set task planner params
    ok = fleet_handle.set_task_planner_params(
        battery_sys,
        motion_sink,
        ambient_sink,
        tool_sink,
        recharge_threshold,
        recharge_soc,
        drain_battery,
        finishing_request)
    assert ok, ("Unable to set task planner params")

    task_capabilities = []
    if fleet_config['task_capabilities']['loop']:
        node.get_logger().info(
            f"Fleet [{fleet_name}] is configured to perform Loop tasks")
        task_capabilities.append(TaskType.TYPE_LOOP)
    if fleet_config['task_capabilities']['delivery']:
        node.get_logger().info(
            f"Fleet [{fleet_name}] is configured to perform Delivery tasks")
        task_capabilities.append(TaskType.TYPE_DELIVERY)
    if fleet_config['task_capabilities']['clean']:
        node.get_logger().info(
            f"Fleet [{fleet_name}] is configured to perform Clean tasks")
        task_capabilities.append(TaskType.TYPE_CLEAN)

    # Callable for validating requests that this fleet can accommodate
    def _task_request_check(task_capabilities, msg: TaskProfile):
        if msg.description.task_type in task_capabilities:
            return True
        else:
            return False

    fleet_handle.accept_task_requests(
        partial(_task_request_check, task_capabilities))

    def _consider(description: dict):
        confirm = adpt.fleet_update_handle.Confirmation()
        confirm.accept()
        return confirm

    # Configure this fleet to perform action category
    if 'action_categories' in task_capabilities_config:
        for cat in task_capabilities_config['action_categories']:
            node.get_logger().info(
                f"Fleet [{fleet_name}] is configured"
                f" to perform action of category [{cat}]")
            fleet_handle.add_performable_action(cat, _consider)

    # Transforms
    rmf_coordinates = config_yaml['reference_coordinates']['rmf']
    robot_coordinates = config_yaml['reference_coordinates']['robot']
    transforms = {
        'rmf_to_robot': nudged.estimate(rmf_coordinates, robot_coordinates),
        'robot_to_rmf': nudged.estimate(robot_coordinates, rmf_coordinates)}
    transforms['orientation_offset'] = \
        transforms['rmf_to_robot'].get_rotation()
    mse = nudged.estimate_error(transforms['rmf_to_robot'],
                                rmf_coordinates,
                                robot_coordinates)
    print(f"Coordinate transformation error: {mse}")
    print("RMF to Robot transform:")
    print(f"    rotation:{transforms['rmf_to_robot'].get_rotation()}")
    print(f"    scale:{transforms['rmf_to_robot'].get_scale()}")
    print(f"    trans:{transforms['rmf_to_robot'].get_translation()}")
    print("Robot to RMF transform:")
    print(f"    rotation:{transforms['robot_to_rmf'].get_rotation()}")
    print(f"    scale:{transforms['robot_to_rmf'].get_scale()}")
    print(f"    trans:{transforms['robot_to_rmf'].get_translation()}")

    def _updater_inserter(cmd_handle, update_handle):
        """Insert a RobotUpdateHandle."""
        cmd_handle.update_handle = update_handle

        def _action_executor(category: str,
                             description: dict,
                             execution:
                             adpt.robot_update_handle.ActionExecution):
            with cmd_handle._lock:
                if len(description) > 0 and\
                        description in cmd_handle.graph.keys:
                    cmd_handle.action_waypoint_index = \
                        cmd_handle.find_waypoint(description).index
                else:
                    cmd_handle.action_waypoint_index = \
                        cmd_handle.last_known_waypoint_index
                cmd_handle.on_waypoint = None
                cmd_handle.on_lane = None
                cmd_handle.action_execution = execution
        # Set the action_executioner for the robot
        cmd_handle.update_handle.set_action_executor(_action_executor)
        if ("max_delay" in cmd_handle.config.keys()):
            max_delay = cmd_handle.config["max_delay"]
            cmd_handle.node.get_logger().info(
                f"Setting max delay to {max_delay}s")
            cmd_handle.update_handle.set_maximum_delay(max_delay)
        if (cmd_handle.charger_waypoint_index <
                cmd_handle.graph.num_waypoints):
            cmd_handle.update_handle.set_charger_waypoint(
                cmd_handle.charger_waypoint_index)
        else:
            cmd_handle.node.get_logger().warn(
                "Invalid waypoint supplied for charger. "
                "Using default nearest charger in the map")
    # Initialize robot API for this fleet
    api = RobotAPI(
        fleet_config['fleet_manager']['broker'],
        fleet_config['fleet_manager']['port'],
        fleet_config['fleet_manager']['iot_endpoint'],
        fleet_config['fleet_manager']['root_ca_path'],
        fleet_config['fleet_manager']['private_key_path'],
        fleet_config['fleet_manager']['cert_path'],
        fleet_config['delivery']['dispenser_req'],
        fleet_config['delivery']['ingestor_req'])

    # Initialize robots for this fleet

    missing_robots = config_yaml['robots']

    # TODO
    finish_ae_topic = "rmf_finish_ae"
    node.get_logger().info(f'finish ae topic : {finish_ae_topic}')

    def _add_fleet_robots():
        robots = {}
        while len(missing_robots) > 0:
            time.sleep(0.2)
            for robot_name in list(missing_robots.keys()):
                node.get_logger().debug(f"Connecting to robot: {robot_name}")
                position = api.position(robot_name)
                # node.get_logger().info(f'[{robot_name}]: position {position[0], position[1], position[2]}')
                if position is None:
                    continue
                if len(position) > 2:
                    node.get_logger().info(f"Initializing robot: {robot_name}")
                    robots_config = config_yaml['robots'][robot_name]
                    rmf_config = robots_config['rmf_config']
                    robot_config = robots_config['robot_config']
                    initial_waypoint = rmf_config['start']['waypoint']
                    initial_orientation = rmf_config['start']['orientation']
                    if initial_waypoint is not None:
                        node.get_logger().info(f'initial_waypoint : {initial_waypoint}')
                        node.get_logger().info(f'initial_orientation:{initial_orientation}')
                    starts = []
                    time_now = adapter.now()

                    if (initial_waypoint is not None) and\
                            (initial_orientation is not None):
                        node.get_logger().info(
                            f"Using provided initial waypoint "
                            "[{initial_waypoint}] "
                            f"and orientation [{initial_orientation:.2f}] to "
                            f"initialize starts for robot [{robot_name}]")
                        # Get the waypoint index for initial_waypoint
                        initial_waypoint_index = nav_graph.find_waypoint(
                            initial_waypoint).index
                        starts = [plan.Start(time_now,
                                             initial_waypoint_index,
                                             initial_orientation)]
                    else:
                        node.get_logger().info(f"Running compute_plan_starts for robot: {robot_name}")
                        starts = plan.compute_plan_starts(
                            nav_graph,
                            rmf_config['start']['map_name'],
                            position,
                        time_now)

                    if starts is None or len(starts) == 0:
                        node.get_logger().error(
                            f"Unable to determine StartSet for {robot_name}")
                        continue

                    robot = RobotCommandHandle(
                        name=robot_name,
                        fleet_name=fleet_name,
                        config=robot_config,
                        node=node,
                        graph=nav_graph,
                        vehicle_traits=vehicle_traits,
                        transforms=transforms,
                        map_name=rmf_config['start']['map_name'],
                        start=starts[0],
                        position=position,
                        charger_waypoint=rmf_config['charger']['waypoint'],
                        update_frequency=rmf_config.get(
                            'robot_state_update_frequency', 1),
                        adapter=adapter,
                        api=api,
                        lane_merge_distance=lane_merge_distance,
                        finish_ae_topic=finish_ae_topic)

                    if robot.initialized:
                        robots[robot_name] = robot
                        # Add robot to fleet
                        fleet_handle.add_robot(robot,
                                               robot_name,
                                               profile,
                                               [starts[0]],
                                               partial(_updater_inserter,
                                                       robot))
                        node.get_logger().info(
                            f"Successfully added new robot: {robot_name}")

                    else:
                        node.get_logger().error(
                            f"Failed to initialize robot: {robot_name}")

                    del missing_robots[robot_name]

                else:
                    pass
                    node.get_logger().debug(
                        f"{robot_name} not found, trying again...")
        return

    add_robots = threading.Thread(target=_add_fleet_robots, args=())
    add_robots.start()
    return adapter, api


# ------------------------------------------------------------------------------
# Main
# ------------------------------------------------------------------------------
def main(argv=sys.argv):
    # Init rclpy and adapter
    rclpy.init(args=argv)
    adpt.init_rclcpp()

    node = rclpy.node.Node(f'wm_robot_command_handle')

    node.declare_parameter('nav_graph_path', "/home/wm-server2/rmf_ws/src/mqtt_rmf/fleet-adapter/fleet_adapter_mqtt/map/turtlebot_world/0.yaml")
    node.declare_parameter('config_file_path', "/home/wm-server2/rmf_ws/src/mqtt_rmf/fleet-adapter/fleet_adapter_mqtt/config/config_wm_fleet.yaml")

    nav_graph_path = node.get_parameter('nav_graph_path').value
    config_file_path = node.get_parameter('config_file_path').value
    use_sim_time = node.get_parameter('use_sim_time').value

    node.get_logger().info(f"Starting wm_robot fleet adapter...")

    # Load config and nav graph yamls
    with open(config_file_path, "r") as f:
        config_yaml = yaml.safe_load(f)

    # Enable sim time for testing offline
    param = Parameter("use_sim_time", Parameter.Type.BOOL, use_sim_time)
    node.set_parameters([param])


    node.get_logger().info("right before initialize_fleet")
    adapter, api = initialize_fleet(
        config_yaml,
        nav_graph_path,
        node,
        use_sim_time)

    # TODO : look at this again. I am not sure how I should handle this. delivery not needed for wm
    dis_res_topic = config_yaml['rmf_fleet']['delivery']['dispenser_res']
    ing_res_topic = config_yaml['rmf_fleet']['delivery']['ingestor_res']
    #Init delivery
    delivery_task = DeliveryTask("delivery_task", api, dis_res_topic, ing_res_topic)
    # Create executor for the command handle node
    rclpy_executor = rclpy.executors.SingleThreadedExecutor()
    rclpy_executor.add_node(node)
    rclpy_executor.add_node(delivery_task)

    # Start the fleet adapter
    rclpy_executor.spin()

    # Shutdown
    node.destroy_node()
    rclpy_executor.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
